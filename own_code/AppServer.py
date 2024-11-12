from flask import Flask, render_template, request, Response
import threading
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WebInterface:
    def __init__(self) -> None:
        # initialize a flask object
        self.app = Flask(__name__)
        self.register_endpoints()
        # initialize the output frame and a lock used to ensure thread-safe
        # exchanges of the output frames (useful when multiple browsers/tabs
        # are viewing the stream)
        self.outputFrame = None
        self.lock = threading.Lock()
        # initialize the video stream and allow the camera sensor to
        # warmup
        time.sleep(2.0)

    def register_endpoints(self):
        self.app.add_url_rule("/", view_func=self.index)
        self.app.add_url_rule("/index", view_func=self.index)
        self.app.add_url_rule("/video_feed", view_func=self.video_feed)
        self.app.add_url_rule("/process_button_click/<command_string>", view_func=self.process_button_click)

    @staticmethod
    def index():
        # return the rendered template
        return render_template("index.html")

    def video_feed(self):
        # return the response generated along with the specific media
        # type (mime type)
        return Response(self.generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @staticmethod
    def process_button_click(command_string):
        print('command received:' + command_string)
        return Response()

    def generate(self):
        # loop over frames from the output stream
        while True:
            # wait until the lock is acquired
            with self.lock:
                # check if the output frame is available, otherwise skip
                # the iteration of the loop
                if self.outputFrame is None:
                    continue
                # encode the frame in JPEG format
                # (flag, encodedImage) = cv2.imencode(".jpg", self.outputFrame)
                encodedImage = self.outputFrame
                # ensure the frame was successfully encoded
                # if not flag:
                #     continue
            # yield the output frame in the byte format
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

    def run(self, ip_adress, port, debug=True, threaded=True, use_reloader=False):
        self.app.run(ip_adress, port, debug=debug, threaded=threaded, use_reloader=use_reloader)


if __name__ == "__main__":
    app = WebInterface()
    app.run(ip_adress="0.0.0.0", port=4664)
