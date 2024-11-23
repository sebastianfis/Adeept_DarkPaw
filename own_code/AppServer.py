import logging

# Import necessary libraries
import queue
from flask import Flask, jsonify, render_template, request, Response, send_from_directory, url_for
from werkzeug.serving import make_server
import io
import signal
import socketserver
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


HOST = "0.0.0.0"
WEB_PORT = 4664
app = Flask(__name__, static_url_path='')
directory_path = '/home/pi/Adeept_DarkPaw/own_code/static/'
command_queue = queue.Queue()


class WebServerThread(Thread):
    """
    Class to make the launch of the flask server non-blocking.
    Also adds shutdown functionality to it.
    """
    def __init__(self, app, host, port):
        Thread.__init__(self)
        self.srv = make_server(host, port, app)
        self.ctx = app.app_context()
        self.ctx.push()
        self.cmd_queue = command_queue

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

@app.route('/')
def index():
    """Render the index.html template on the root URL."""
    return render_template('index.html')


@app.route('/process_button_click/<command_string>')
def process_button_click(command_string):
    command_queue.put(command_string)
    # print('command received:' + command_string)
    return Response()


@app.route("/<string:page_name>")
def page(page_name):
    return render_template("{}".format(page_name))


@app.route("/static/<path:path>")
def send_static(path):
    return send_from_directory(directory_path, path)


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


if __name__ == '__main__':
    # registering both types of signals
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # firing up the video camera (pi camera)
    camera = Picamera2()
    camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
    camera.configure(camera.create_video_configuration(main={"size": (800, 600)}, raw={'format': 'SGRBG10'}))
    camera.start()
    time.sleep(1)
    #
    # metadata = camera.capture_metadata()
    # control_values = {c: metadata[c] for c in ["ExposureTime", "AnalogueGain", "ColourGains"]}
    # print(control_values)

    output = StreamingOutput()
    camera.start_recording(JpegEncoder(), FileOutput(output))
    logging.info("Started recording with picamera2")
    STREAM_PORT = 4665
    stream = StreamingServer((HOST, STREAM_PORT), StreamingHandler)

    # starting the video streaming server
    streamserver = Thread(target=stream.serve_forever)
    streamserver.start()
    logging.info("Started stream server for picamera2")

    # starting the web server
    webserver = WebServerThread(app, HOST, WEB_PORT)
    webserver.start()
    logging.info("Started Flask web server")

    # and run it indefinitely
    while not keyboard_trigger.is_set():
        time.sleep(0.5)

    # until some keyboard event is detected
    logging.info("Keyboard event detected")

    # trigger shutdown procedure
    webserver.shutdown()
    camera.stop_recording()
    stream.shutdown()

    # and finalize shutting them down
    webserver.join()
    streamserver.join()
    logging.info("Stopped all threads")

#
# class WebInterface:
#     def __init__(self) -> None:
#         # initialize a flask object
#         self.app = Flask(__name__)
#         self.sio_instance = SocketIO(self.app)
#         self.register_endpoints()
#         # initialize the output frame and a lock used to ensure thread-safe
#         # exchanges of the output frames (useful when multiple browsers/tabs
#         # are viewing the stream)
#         self.outputFrame = None
#         self.lock = threading.Lock()
#         # initialize the video stream and allow the camera sensor to
#         # warmup
#         time.sleep(2.0)
#
#     def register_endpoints(self):
#         self.app.add_url_rule("/", view_func=self.index)
#         self.app.add_url_rule("/index", view_func=self.index)
#         # self.app.add_url_rule("/video_feed", view_func=self.video_feed)
#         self.app.add_url_rule("/process_button_click/<command_string>", view_func=self.process_button_click)
#
#     @staticmethod
#     def index():
#         # return the rendered template
#         return render_template("index.html")
#
#     def video_feed(self):
#         # return the response generated along with the specific media
#         # type (mime type)
#         # return Response(self.generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
#         self.sio_instance.emit('send', self.generate())
#
#     @staticmethod
#     def process_button_click(command_string):
#         print('command received:' + command_string)
#         return Response()
#
#     def generate(self):
#         # loop over frames from the output stream
#         while True:
#             # wait until the lock is acquired
#             with self.lock:
#                 # check if the output frame is available, otherwise skip
#                 # the iteration of the loop
#                 if self.outputFrame is None:
#                     continue
#                 # encode the frame in JPEG format
#                 (flag, frame) = cv2.imencode(".jpg", self.outputFrame)
#                 # encodedImage = self.outputFrame
#                 # ensure the frame was successfully encoded
#                 if not flag:
#                     continue
#             # yield the output frame in the byte format
#             yield base64.encodebytes(frame[1].tobytes()).decode("utf-8")
#             # yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
#             #        bytearray(encodedImage) + b'\r\n')
#
#     def run(self, ip_adress, port, debug=True, threaded=True, use_reloader=False):
#         self.app.run(ip_adress, port, debug=debug, threaded=threaded, use_reloader=use_reloader)
#
#
# if __name__ == "__main__":
#     app = WebInterface()
#     app.run(ip_adress="0.0.0.0", port=4664)
