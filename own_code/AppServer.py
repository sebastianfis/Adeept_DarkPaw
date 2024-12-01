import logging

# Import necessary libraries
from queue import Queue
from flask import Flask, render_template, Response, send_from_directory
from werkzeug.serving import make_server
import io
import socketserver
from threading import Condition, Thread, Event
from http import server
from picamera2 import MappedArray, Picamera2
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

class WebServerThread(Thread):
    """
    Class to make the launch of the flask server non-blocking.
    Also adds shutdown functionality to it.
    """
    def __init__(self, command_q: Queue, host="0.0.0.0", port=4664,
                 directory_path='/home/pi/Adeept_DarkPaw/own_code/static/'):
        Thread.__init__(self)
        self.app = Flask(__name__, static_url_path='')
        self.srv = make_server(host, port, self.app)
        self.ctx = self.app.app_context()
        self.ctx.push()
        self.cmd_queue = command_q
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
# TODO: 2. program value update route


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
    output = StreamingOutput()

    def __init__(self, request: bytes, client_address: tuple[str, int],
                 server: socketserver.BaseServer):
        super().__init__(request, client_address, server)

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
                    with self.output.condition:
                        self.output.condition.wait()
                        frame = self.output.frame
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


def setup_webserver(command_q: Queue, camera_instance: Picamera2, host="0.0.0.0", port=4664, video_port=4665):
    # registering both types of signals
    videostream = StreamingServer((host, video_port), StreamingHandler)
    camera_instance.start_recording(JpegEncoder(), FileOutput(StreamingHandler.output))

    stream_server = Thread(target=videostream.serve_forever)
    stream_server.start()
    logging.info("Started stream server for picamera2")

    # starting the web server
    web_server = WebServerThread(command_q, host=host, port=port)
    web_server.start()
    logging.info("Started Flask web server")

    return videostream, stream_server, web_server


if __name__ == '__main__':
    command_queue = Queue()

    camera = Picamera2()
    video_w, video_h = 800, 600
    camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
    camera_config = camera.create_video_configuration(main={'size': (video_w, video_h), 'format': 'XRGB8888'},
                                                      raw={'format': 'SGRBG10'}, controls={'FrameRate': 30})
    camera.preview_configuration.align()
    camera.configure(camera_config)
    stream, streamserver, webserver = setup_webserver(command_queue, camera)

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
    streamserver.join()
    logging.info("Stopped all threads")
