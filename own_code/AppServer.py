import logging

# Import necessary libraries
from queue import Queue
from flask import Flask, jsonify, render_template, request, Response, send_from_directory, url_for
from werkzeug.serving import make_server
import io
import signal
import socketserver
import cv2
from threading import Condition, Thread, Event
from http import server
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from libcamera import controls
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# for triggering the shutdown procedure when a signal is detected
keyboard_trigger = Event()


def signal_handler(signal, frame):
    logging.info('Signal detected. Stopping threads.')
    keyboard_trigger.set()


#######################
#   Web Server Stuff  #
#######################


# HOST = "0.0.0.0"
# WEB_PORT = 4664
# app = Flask(__name__, static_url_path='')
# directory_path = '/home/pi/Adeept_DarkPaw/own_code/static/'
# command_queue = queue.Queue()


class WebServerThread(Thread):
    """
    Class to make the launch of the flask server non-blocking.
    Also adds shutdown functionality to it.
    """
    def __init__(self, command_queue: Queue, host="0.0.0.0", port=4664,
                 directory_path='/home/pi/Adeept_DarkPaw/own_code/static/'):
        Thread.__init__(self)
        self.app = Flask(__name__, static_url_path='')
        self.srv = make_server(host, port, self.app)
        self.ctx = self.app.app_context()
        self.ctx.push()
        self.cmd_queue = command_queue
        self.directory_path = directory_path
        self.register_endpoints()

    def register_endpoints(self):
        self.app.add_url_rule("/", view_func=self.index)
        self.app.add_url_rule("/index", view_func=self.index)
        self.app.add_url_rule("/<string:page_name>", view_func=self.page)
        self.app.add_url_rule("/static/<path:path>", view_func=self.send_static)
        self.app.add_url_rule("/process_button_click/<command_string>", view_func=self.process_button_click)

    @staticmethod
    def index():
        # return the rendered template
        return render_template("index.html")

    def process_button_click(self, command_string):
        self.cmd_queue.put(command_string)
        # print('command received:' + command_string)
        return Response()

    @staticmethod
    def page(page_name):
        return render_template("{}".format(page_name))

    def send_static(self, path):
        return send_from_directory(self.directory_path, path)

    def run(self):
        logging.info('Starting Flask server')
        self.srv.serve_forever()

    def shutdown(self):
        logging.info('Stopping Flask server')
        self.srv.shutdown()

# TODO: 1. Add velocity setting bar
# TODO: 2. Check if this whole mess can be wrapped in a class...
# TODO: 3. put all the stuff in main into dedicated function (e.g. setup server)
# TODO: 4. program value update route
# TODO: 5. Start this from DMN together with all the other code...


#############################
#   Video Streaming Stuff   #
#############################

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    """
    Implementing GET request for the video stream.
    """

    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def setup_webserver(command_queue: Queue, output: StreamingOutput, host="0.0.0.0", port=4664, video_port=4665):
    # registering both types of signals
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


    # starting the video streaming server
    stream = StreamingServer((host, video_port), StreamingHandler)
    streamserver = Thread(target=stream.serve_forever)
    streamserver.start()
    logging.info("Started stream server for picamera2")

    # starting the web server
    webserver = WebServerThread(command_queue, host=host, port=port)
    webserver.start()
    logging.info("Started Flask web server")

    return stream, streamserver, webserver


def capture_array_from_camera(cam: Picamera2, out: StreamingOutput, fps=30):
    last_exec_time = time.time_ns() / 1e6
    while True and not keyboard_trigger.is_set():
        now_time = time.time_ns()/1e6
        # limit frame rate:
        if (now_time-last_exec_time) >= 1000/fps:
            time.sleep(0.2)
            full_frame = cam.capture_array('main')
            cv2.putText(img=full_frame,
                        text='FPS = {:04.1f}'.format(1000/(now_time-last_exec_time)),
                        org=(full_frame.shape[1] - 120, 20),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.5,
                        color=(255, 255, 255),
                        thickness=1,
                        lineType=cv2.LINE_AA)
            r, buf = cv2.imencode(".jpg", full_frame)
            out.write(buf.tobytes())
            cv2.waitKey(1)
            last_exec_time = now_time


if __name__ == '__main__':
    command_queue = Queue()
    output = StreamingOutput()
    stream, streamserver, webserver = setup_webserver(command_queue, output)

    # firing up the video camera (pi camera)
    camera = Picamera2()
    camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
    camera_config = camera.create_video_configuration(main={'size': (800, 600), 'format': 'RGB888'},
                                                      raw={'format': 'SGRBG10'}, controls={'FrameRate': 30})

    camera.align_configuration(camera_config)
    camera.configure(camera_config)

    camera.start()
    time.sleep(1)

    # camera.start_recording(JpegEncoder(), FileOutput(output))
    cam_thread = Thread(target=capture_array_from_camera, args=(camera, output))
    cam_thread.start()
    logging.info("Started recording with picamera2")

    # and run it indefinitely
    while not keyboard_trigger.is_set():
        time.sleep(0.5)

    # until some keyboard event is detected
    logging.info("Keyboard event detected")

    # trigger shutdown procedure
    webserver.shutdown()
    stream.shutdown()
    camera.stop()

    # and finalize shutting them down
    webserver.join()
    cam_thread.join()
    streamserver.join()
    logging.info("Stopped all threads")


