import logging
from detection_engine import DetectionEngine
from queue import Queue, Empty
from multiprocessing import Process, SimpleQueue
from threading import Timer, Event, Thread
from AdditionalEquipment import led_worker, distance_sensor_worker, get_cpu_tempfunc, get_cpu_use, get_ram_info
from MotionControl import MotionController
import json
import time


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# TODO: Add behaviour


class DefaultModeNetwork:
    def __init__(self, detector: DetectionEngine, data_queue: Queue, command_queue: Queue):
        self.command_queue = command_queue
        self.data_queue = data_queue
        self.LED_queue = SimpleQueue()
        self.led_stopped = Event()
        self.led_stopped.clear()
        self.distance_queue = SimpleQueue()
        self.run_distance_measurement = Event()
        self.run_distance_measurement.set()
        self.data_dict = {}
        self.motion_controller = MotionController()
        self.mode = 'remote_controlled'
        self.current_detections = {}
        self.last_exec_time = time.perf_counter_ns()
        self.min_detect_velocity = 0.28  # min detectable veloyitc in m/s: Calculated from regluar walking velocity
                                         # 6 km/h / 3.6 km/h m/s
        self.movement_measurement_timer = 200  # Check for movement all 200 ms
        self.selected_target = None
        self.last_dist_measuremnt = 1e20
        self.target_centered = False
        self.target_moving = False
        self.target_drop_timer = Timer(3, self.drop_target)  # 3 seconds to re-acquire a lost target
        self.highest_id = 0
        self.turn_around_time = 2.4781 # How many seconds it takes the robot theoretically to do half a turn at full velocity

        # start up lighting
        self.led_process = Process(target=led_worker, args=(self.LED_queue, self.led_stopped))
        self.led_process.start()

        # start up distance measurement
        self.dist_measure_process = Process(target=distance_sensor_worker, args=(self.distance_queue,
                                                                                 self.run_distance_measurement))
        self.dist_measure_process.start()
        self.detector = detector

    def run(self):
        self.LED_queue.put(('all_good', True))
        while self.detector.running:
            now_time = time.time_ns()
            if not self.distance_queue.empty():
                self.last_dist_measuremnt = round(self.distance_queue.get(), 2)
            self.data_dict = {'Distance': "{0:.2f}".format(self.last_dist_measuremnt),
                              'CPU_temp': get_cpu_tempfunc(),
                              'CPU_load': get_cpu_use(),
                              'RAM_usage': get_ram_info()}
            if self.data_queue.full():
                try:
                    self.data_queue.get_nowait()  # Drop the oldest frame to prevent queue backup
                except Empty:
                    pass
            self.data_queue.put_nowait(self.data_dict)
            detections = self.detector.get_results(as_dict=True)
            if detections is not None and self.mode not in ['dance', 'stabilize', 'remote_controlled']:
                self.select_target(detections)
                self.update_detection_counter(detections)
                self.auto_drop_target(detections)
            if not self.command_queue.empty():
                command_str = self.command_queue.get()
                if 'mode_select:' in command_str:
                    new_mode = command_str.split(':')[1]
                    self.mode = new_mode
                    logging.info('mode selected: ' + new_mode)
                    if new_mode in ['dance', 'stabilize']:
                        if new_mode == 'dance':
                            self.LED_queue.put(('disco', False))
                        else:
                            self.LED_queue.put(('all_good', True))
                        self.motion_controller.execute_command(new_mode)
                    elif new_mode == 'remote_controlled':
                        self.LED_queue.put(('remote_controlled', True))
                        self.motion_controller.issue_reset_command()
                    elif new_mode == 'patrol':
                        self.LED_queue.put(('police', False))
                    else:
                        self.LED_queue.put(('all_good', True))
                elif self.mode == 'remote_controlled':
                    self.motion_controller.execute_command(command_str)
                # TODO: Add code for patrol mode and autonomous mode
            self.last_exec_time = now_time
            time.sleep(0.01)

    def load_class_occurences(self):
        # try to load previosly counted detections as dict:
        try:
            with open('class_occurence_counter.json') as f:
                class_occurences = json.load(f)
        # if that does not exist, create new dict
        except FileNotFoundError:
            class_occurences = {}
            for ii, item in enumerate(self.detector.class_names):
                class_occurences[str(ii)] = {'name': item,
                                             'counter': 0}
        return class_occurences

    def update_class_occurences(self, new_detection):
        logging.info('new object detected:')
        logging.info(new_detection)
        class_occurences = self.load_class_occurences()
        class_occurences[str(new_detection['class'])]['counter'] += 1
        with open('class_occurence_counter.json', 'w') as f:
            json.dump(class_occurences, f)

    def update_detection_counter(self, detections):
        if detections:
            # Without previous detections: Update list with all detections
            if not self.current_detections:
                for key in detections.keys():
                    new_detection = detections[key]
                    self.update_class_occurences(new_detection)
            # With previous detections: Only count new detecion if bigger than prev. max tracker id!
            elif max(detections.keys()) > self.highest_id:
                self.highest_id = max(detections.keys())
                new_detection = detections[max(detections.keys())]
                self.update_class_occurences(new_detection)
            self.current_detections = detections

    def select_target(self, detections):
        # Only choose new target if old one is dropped and detections contain something!
        if self.selected_target is None and detections:
            class_occurences = self.load_class_occurences()
            cur_target = {'conf': 0}
            target_occurence = 1e20
            for target_id in detections.keys():
                if int(class_occurences[str(detections[target_id]['class'])]['counter']) < target_occurence:
                    target_occurence = int(class_occurences[str(detections[target_id]['class'])]['counter'])
                    cur_target = detections[target_id]
                    cur_target['id'] = target_id
                elif int(class_occurences[str(detections[target_id]['class'])]['counter']) == target_occurence:
                    if detections[target_id]['conf'] > cur_target['conf']:
                        cur_target = detections[target_id]
                        cur_target['id'] = target_id
            self.selected_target = cur_target
            self.LED_queue.put(('yellow_alert', True))
            logging.info('target acquired:' + str(cur_target))

    def drop_target(self):
        logging.info('target dropped:' + str(self.selected_target))
        self.selected_target = None
        self.target_centered = False
        self.target_moving = False
        if self.mode == 'patrol':
            self.LED_queue.put(('police', False))
        else:
            self.LED_queue.put(('all_good', True))

    def auto_drop_target(self, detections):
        if self.selected_target is not None:
            if self.selected_target['id'] in detections and self.target_drop_timer.is_alive():
                # reset timer
                try:
                    self.target_drop_timer.cancel()
                except Exception:
                    pass
                finally:
                    self.target_drop_timer.join()
            elif not self.target_drop_timer.is_alive():
                # only start timer, if it is not already running!
                self.target_drop_timer = Timer(3, self.drop_target)  # 3 seconds to re-acquire a lost target
                self.target_drop_timer.start()

    def look_at_target(self, deadband=50, focus_y=False):
        if self.selected_target is not None:
            centroid_x = (self.selected_target['bbox'][0] + self.selected_target['bbox'][2]) / 2
            centroid_y = (self.selected_target['bbox'][1] + self.selected_target['bbox'][3]) / 2
            if centroid_x < (self.detector.video_w - deadband) / 2:
                self.target_centered = False
                if self.motion_controller.last_command != 'turn_left':
                    self.motion_controller.execute_command('turn_left')
            elif centroid_x > (self.detector.video_w + deadband) / 2:
                self.target_centered = False
                if self.motion_controller.last_command != 'turn_right':
                    self.motion_controller.execute_command('turn_right')
            else:
                if self.motion_controller.last_command != 'stop':
                    self.motion_controller.execute_command('stop')
                self.target_centered = True
            if self.target_centered and centroid_y > 2 / 3 * self.detector.video_h and focus_y and \
                    self.motion_controller.last_command != 'look_down':
                self.motion_controller.execute_command('look_down')
            elif self.target_centered and centroid_y < 1 / 3 * self.detector.video_h and focus_y and \
                    self.motion_controller.last_command != 'look_up':
                self.motion_controller.execute_command('look_up')

    def check_if_moving_target(self, now_time):
        if self.selected_target and self.target_centered:
            if (now_time - self.last_exec_time) > 1e6 * self.movement_measurement_timer:
                # TODO: Add code for motion detection
                pass

    def approach_target(self, target_distance=50, delta=2):
        if self.selected_target and self.target_centered:
            if self.last_dist_measuremnt > target_distance + delta:
                if self.motion_controller.last_command != 'move_forward':
                    self.motion_controller.execute_command('move_forward')
            elif self.last_dist_measuremnt < target_distance - delta:
                if self.motion_controller.last_command != 'move_backward':
                    self.motion_controller.execute_command('move_backward')
            else:
                if self.motion_controller.last_command != 'stop':
                    self.motion_controller.execute_command('stop')

    def shutdown(self):
        # for triggering the shutdown procedure when a signal is detected
        # trigger shutdown procedure
        self.run_distance_measurement.clear()
        self.led_stopped.set()
        time.sleep(0.01)

        # and finalize shutting them down
        self.dist_measure_process.join()
        self.led_process.join()
        logging.info("Stopped all processes")


if __name__ == '__main__':
    try:
        data_queue = Queue(maxsize=2)
        command_queue = Queue(maxsize=1)
        detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov11m.hef',
                                   score_thresh=0.7,
                                   max_detections=3)
        dmn = DefaultModeNetwork(detector, data_queue, command_queue)
        dmn_thread = Thread(target=dmn.run, daemon=True)
        dmn_thread.start()

    except KeyboardInterrupt:
        logger.info("🛑 KeyboardInterrupt received. Exiting...")
        dmn.shutdown()
        dmn_thread.join(timeout=2)

