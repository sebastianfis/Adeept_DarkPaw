import logging
from AppServer import setup_webserver, StreamingOutput
from detection_engine import DetectionEngine
from queue import Queue
import threading

logging.basicConfig(level=None) # logging.INFO)
logger = logging.getLogger(__name__)


def thread():
    while True:
        if not command_queue.empty():
            command_str = command_queue.get()
            print('command received:' + command_str)


if __name__ == '__main__':
    command_queue = Queue()

    detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov10b.hef',
                               score_thresh=0.65,
                               max_detections=3)
    stream, streamserver, webserver = setup_webserver(command_queue, detector.camera)

    while True:
        try:
            detector.run_inference()
            if not command_queue.empty():
                command_str = command_queue.get()
                print('command received:' + command_str)
        except KeyboardInterrupt:
            logging.info("Keyboard event detected")

            # trigger shutdown procedure
            webserver.shutdown()
            stream.shutdown()
            detector.camera.stop()

            # and finalize shutting them down
            webserver.join()
            streamserver.join()
            logging.info("Stopped all threads")


