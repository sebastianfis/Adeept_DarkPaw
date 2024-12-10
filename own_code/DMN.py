import logging
from AppServer import setup_webserver
from detection_engine import DetectionEngine
from queue import Queue
from threading import Thread, Event, Timer
from AdditionalEquipment import LED, DistSensor, get_cpu_tempfunc, get_cpu_use, get_ram_info
from MotionControl import MotionController
import signal
import json
import RPi.GPIO as GPIO

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logging.getLogger('werkzeug').disabled = True

GPIO.setmode(GPIO.BCM)

# TODO: Add behaviour


class DefaultModeNetwork:
    def __init__(self):
        self.command_queue = Queue()
        self.data_queue = Queue()
        self.data_dict = {}
        self.dist_sensor = DistSensor()
        self.dist_sensor.enable_cont_meaurement()
        self.motion_controller = MotionController()
        self.mode = 'remote_controlled'
        self.current_detections = {}
        self.selected_target = None
        self.target_drop_timer = Timer(3, self.drop_target)  # 3 seconds to re-acquire a lost target

        # start up lighting
        self.led_instance = LED()
        self.lights_thread = Thread(target=self.led_instance.run_lights)
        self.lights_thread.start()

        # start up distance measurement
        self.dist_measure_thread = Thread(target=self.dist_sensor.measure_cont)
        self.dist_measure_thread.start()

        # start up detection engine incl. camera
        self.detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov10b.hef',
                                        score_thresh=0.70,
                                        max_detections=3)
        self.keyboard_trigger = Event()

        # start up web interface
        self.stream, self.streamserver, self.webserver = setup_webserver(self.command_queue,
                                                                         self.data_queue,
                                                                         self.detector.camera)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signal, frame):
        logging.info('Signal detected. Stopping threads.')
        self.keyboard_trigger.set()

    def run(self):
        self.led_instance.light_setter('all_good', breath=True)
        while not self.keyboard_trigger.is_set():
            self.detector.run_inference()
            self.data_dict = {'Distance': "{0:.2f}".format(round(self.dist_sensor.read_last_measurement(), 2)),
                              'CPU_temp': get_cpu_tempfunc(),
                              'CPU_load': get_cpu_use(),
                              'RAM_usage': get_ram_info()}
            self.data_queue.put(self.data_dict)
            detections = self.detector.get_results(as_dict=True)
            # FIXME: This does not work yet!
            self.select_target(detections)
            self.update_detection_counter(detections)
            # logging.info(detections)
            if not self.command_queue.empty():
                command_str = self.command_queue.get()
                if 'mode_select:' in command_str:
                    new_mode = command_str.split(':')[1]
                    self.mode = new_mode
                    logging.info('mode selected: ' + new_mode)
                    if new_mode in ['dance', 'stabilize']:
                        self.motion_controller.execute_command(new_mode)
                    elif new_mode == 'remote_controlled':
                        self.motion_controller.issue_reset_command()
                elif self.mode == 'remote_controlled':
                    self.motion_controller.execute_command(command_str)

        # until some keyboard event is detected
        self.shutdown()

    def load_class_occurences(self):
        # try to load previosly counted detections as dict:
        try:
            with open('class_occurence_counter.json') as f:
                class_occurences = json.load(f)
        # if that does not exist, create new dict
        except FileNotFoundError:
            class_occurences = {}
            for ii, item in enumerate(self.detector.class_names):
                class_occurences[ii] = {'name': item,
                                        'counter': 0}
        return class_occurences

    def update_detection_counter(self, detections):
        # Only count new detecion if bigger than prev. max tracker id!
        if not self.current_detections.keys() or max(detections.keys()) > max(self.current_detections.keys()):
            new_detection = detections[max(detections.keys())]
            logging.info('new object detected:')
            logging.info(new_detection)
            class_occurences = self.load_class_occurences()
            class_occurences[new_detection['class']]['counter'] += 1
            with open('class_occurence_counter.json', 'w') as f:
                json.dump(class_occurences, f)
        self.current_detections = detections

    def select_target(self, detections):
        # Only choose new target if old one is dropped and detections contain something!
        if self.selected_target is None and detections:
            class_occurences = self.load_class_occurences()
            cur_target = {'conf': 0}
            target_occurence = 1e20
            for target_id in detections.keys():
                if class_occurences[detections[target_id]['class']]['counter'] < target_occurence:
                    target_occurence = class_occurences[detections[target_id]['class']]['counter']
                    cur_target = detections[target_id]
                    cur_target['id'] = target_id
                elif class_occurences[detections[target_id]['class']]['counter'] == target_occurence:
                    if detections[target_id]['conf'] > cur_target['conf']:
                        cur_target = detections[target_id]
                        cur_target['id'] = target_id
            self.selected_target = cur_target
            logging.info('target acquired:' + str(cur_target))

    def drop_target(self):
        logging.info('target dropped:' + str(self.selected_target))
        self.selected_target = None

    def auto_drop_target(self, detections):
        if self.selected_target is not None and self.selected_target['id'] in detections:
            # reset timer
            self.target_drop_timer.cancel()
        elif not self.target_drop_timer.is_alive():
            # only start timer, if it is not already running!
            self.target_drop_timer.start()

    def look_at_target(self, deadband=50):
        # TODO: Add target focus
        pass

    def shutdown(self):
        # for triggering the shutdown procedure when a signal is detected
        # trigger shutdown procedure
        self.webserver.shutdown()
        self.stream.shutdown()
        self.detector.camera.stop()
        self.dist_sensor.disable_cont_meaurement()
        self.led_instance.shutdown()

        # and finalize shutting them down
        self.webserver.join()
        self.streamserver.join()
        self.dist_measure_thread.join()
        self.lights_thread.join()
        GPIO.cleanup()
        logging.info("Stopped all threads")


class Behaviour:
    def __init__(self, name: str):
        pass


if __name__ == '__main__':
    dmn = DefaultModeNetwork()
    dmn.run()



