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
    # eval_thread = threading.Thread(target=thread)
    # eval_thread.start()
    detector = DetectionEngine(model_path='/models/yolov8m.hef', score_thresh=0.5)
    socketio_instance, flask_app = setup_server(command_queue, detector, logger=logger)
    eval_thread = threading.Thread(target=detector.capture_frames, args=[socketio_instance])
    eval_thread.Daemon =True
    eval_thread.start()
    #socketio_instance.start_background_task(detector.capture_frames, socketio=socketio_instance)
    socketio_instance.run(flask_app, host='0.0.0.0', port=4664)






