# Import necessary libraries
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import base64
from queue import Queue
from logging import Logger
import eventlet
from detection_engine import DetectionEngine
# import os, random # testing only


def setup_server(command_queue: Queue, detection_engine: DetectionEngine, logger: Logger):
    # Initialize Flask application and Socket.IO
    app = Flask(__name__)
    socketio = SocketIO(app, logger=logger, async_mode='eventlet')

    @app.route('/')
    def index():
        """Render the index.html template on the root URL."""
        return render_template('index.html')

    @app.route('/process_button_click/<command_string>')
    def process_button_click(command_string):
        command_queue.put(command_string)
        print('command received:' + command_string)
        return Response()

    @socketio.on("request-frame")
    def camera_frame_requested(message):
        frame = detection_engine.get_frame()
        if frame is not None:
            emit("new-frame", {
                "base64": base64.b64encode(frame).decode("ascii")
            }, broadcast=True)

    return socketio, app


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
#
# if __name__ == '__main__':
#     socketio_instance, flask_app = setup_server()
#     socketio_instance.start_background_task(capture_frames, socketio=socketio_instance)
#     socketio_instance.run(flask_app, host='0.0.0.0', port=4664)
#
#