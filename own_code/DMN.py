import logging
from AppServer import setup_server
from detection_engine import DetectionEngine
from queue import Queue
import threading

logging.basicConfig(level=None)#logging.INFO)
logger = logging.getLogger(__name__)


def thread():
    while True:
        if not command_queue.empty():
            command_str = command_queue.get()
            print('command received:' + command_str)


if __name__ == '__main__':
    command_queue = Queue()
    results_queue = Queue()

    detector = DetectionEngine(model_path='/models/yolov8m.hef', score_thresh=0.5)
    eval_thread = threading.Thread(target=detector.run_inference, args=[results_queue])
    eval_thread.Daemon = True
    eval_thread.start()







