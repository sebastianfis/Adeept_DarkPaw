import logging
from detection_engine import DetectionEngine
from queue import Queue, Empty
from multiprocessing import Process, SimpleQueue
from threading import Timer, Event, Thread
from AdditionalEquipment import led_worker, distance_sensor_worker, get_cpu_tempfunc, get_cpu_use, get_ram_info
from MotionControl import motion_control_worker
import json
import time
import numpy as np


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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
        self.motion_command_queue = SimpleQueue()
        self.motion_controller_stopped = Event()
        self.motion_controller_stopped.clear()
        # self.motion_controller = MotionController()
        self.mode = 'remote_controlled'
        self.current_detections = {}
        self.centroid_x_list = None
        self.distance_list = None
        self.pause_for_motion_detection = False
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

        # start up lighting
        self.led_process = Process(target=led_worker, args=(self.LED_queue, self.led_stopped))
        self.led_process.start()

        # start up distance measurement
        self.dist_measure_process = Process(target=distance_sensor_worker, args=(self.distance_queue,
                                                                                 self.run_distance_measurement))
        self.dist_measure_process.start()

        # start up motion control interface
        self.motion_control_process = Process(target=motion_control_worker, args=(self.motion_command_queue,
                                                                                  self.motion_controller_stopped))
        self.motion_control_process.start()

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
                    self.data_queue.get_nowait()  # Drop the oldest data to prevent queue backup
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
                        self.motion_command_queue.put(new_mode)
                    elif new_mode == 'remote_controlled':
                        self.LED_queue.put(('remote_controlled', True))
                        self.motion_command_queue.put('stop')
                    elif new_mode == 'patrol':
                        self.LED_queue.put(('police', False))
                    else:
                        self.LED_queue.put(('all_good', True))
                elif self.mode == 'remote_controlled':
                    self.motion_command_queue.put(command_str)
            if self.mode == 'patrol':
                self.go_on_patrol()
            elif self.mode == 'behaviour_model':
                self.execute_behaviour()
            self.check_if_moving_target(now_time)
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
        cur_target = {'conf': 0}
        target_occurence = 1e20
        if detections:
            if self.selected_target is None:
                class_occurences = self.load_class_occurences()
                # Choose target with lowest overall occurance as most interesting one!
                for target_id in detections.keys():
                    current_id_occurence = int(class_occurences[str(detections[target_id]['class'])]['counter'])
                    if current_id_occurence < target_occurence:
                        target_occurence = current_id_occurence
                        cur_target = detections[target_id]
                        cur_target['id'] = target_id
                    # for equal occurences choose target with higher confidence score
                    elif current_id_occurence == target_occurence:
                        if detections[target_id]['conf'] > cur_target['conf']:
                            cur_target = detections[target_id]
                            cur_target['id'] = target_id
                self.selected_target = cur_target
                self.LED_queue.put(('yellow_alert', True))
                logging.info('target acquired:' + str(cur_target))
            # Update current detection for data for selected target (in case of moving target)
            elif self.selected_target is not None and self.selected_target['id'] in detections.keys():
                self.selected_target = detections[self.selected_target['id']]

    def drop_target(self):
        logging.info('target dropped:' + str(self.selected_target))
        self.selected_target = None
        self.target_centered = False
        self.target_moving = False
        self.centroid_x_list = None
        self.distance_list = None
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
        if self.selected_target is not None and not self.pause_for_motion_detection:
            centroid_x = (self.selected_target['bbox'][0] + self.selected_target['bbox'][2]) / 2
            centroid_y = (self.selected_target['bbox'][1] + self.selected_target['bbox'][3]) / 2
            if centroid_x < (self.detector.video_w - deadband) / 2:
                self.target_centered = False
                self.motion_command_queue.put('turn_left')
            elif centroid_x > (self.detector.video_w + deadband) / 2:
                self.target_centered = False
                self.motion_command_queue.put('turn_right')
            else:
                self.motion_command_queue.put('stop')
                self.target_centered = True
            if self.target_centered and centroid_y > 2 / 3 * self.detector.video_h and focus_y:
                self.motion_command_queue.put('look_down')
            elif self.target_centered and centroid_y < 1 / 3 * self.detector.video_h and focus_y:
                self.motion_command_queue.put('look_up')

    def check_if_moving_target(self, now_time):
        # ToDo: Test this!
        while not self.target_centered:
            self.look_at_target() # FIXME: This code part would be blocking and needs rework!!!
        self.pause_for_motion_detection = True
        self.centroid_x_list = []
        self.centroid_y_list = [] # ToDo: Include y-evaluation!
        self.distance_list = []
        if (now_time - self.last_exec_time) > 1e6 * self.movement_measurement_timer:
            if self.selected_target:
                if len(self.centroid_x_list) > 10:
                    self.centroid_x_list.pop(0)
                if len(self.distance_list) > 10:
                    self.centroid_x_list.pop(0)
                centroid_x = (self.selected_target['bbox'][0] + self.selected_target['bbox'][2]) / 2
                self.centroid_x_list.append(centroid_x)
                self.distance_list.append(self.last_dist_measuremnt)
        # time = np.linspace(0, 10 * 1e6 * self.movement_measurement_timer, len(self.distance_list))
        approx_resolution = np.tan(62.2 / 180 * np.pi) * self.last_dist_measuremnt / 100
        if len(self.centroid_x_list) == 10:
            # use mean instead of linear fit to be more resilient vs. drift!
            # v_x = np.polyfit(time, [x * approx_resolution for x in self.centroid_x_list], 1)[0]

            v_x = np.mean([j - i for i, j in zip(self.centroid_x_list[:-1], self.centroid_x_list[1:])]) * approx_resolution
            logger.info(f'calculated x velocity = {v_x} m/s')
        else:
            v_x = 0
        if len(self.distance_list) == 10:
            # use mean instead of linear fit to be more resilient vs. drift!
            # v_z = np.polyfit(time, [z / 100 for z in self.distance_list], 1)[0]
            v_z = np.mean([j - i for i, j in zip(self.distance_list[:-1], self.distance_list[1:])]) / 100
            logger.info(f'calculated z velocity = {v_z} m/s')
        else:
            v_z = 0
        v_total = (v_x**2 + v_z**2)**0.5
        if v_total > self.min_detect_velocity:
            self.target_moving = True
            self.LED_queue.put(('red_alert', True))
        self.pause_for_motion_detection = False

    def go_on_patrol(self):
        # TODO: Add code for patrol mode
        # turn circumference is 173.74 mm * 2 * pi = 1092,64 mm half a turn is then 545.82 mm
        # 4 steps result in a turn of the step_length = 80 mm on the circumference.
        # One step is represented by 8 samples at an update_freq = 50 Hz.
        # Therefore: turn_velocity = step_length / 4 / 8 * update_freq / velocity percentage = 125 mm/s
        # A 180° turn is executed in about 4.4 seconds / velocity percentage
        pass

    def execute_behaviour(self):
        # TODO: Add code for autonomous mode
        pass

    def approach_target(self, target_distance=50, delta=2):
        if self.selected_target and self.target_centered and not self.pause_for_motion_detection:
            if self.last_dist_measuremnt > target_distance + delta:
                self.motion_command_queue.put('move_forward')
            elif self.last_dist_measuremnt < target_distance - delta:
                self.motion_command_queue.put('move_backward')
            else:
                self.motion_command_queue.put('stop')

    def shutdown(self):
        # for triggering the shutdown procedure when a signal is detected
        # trigger shutdown procedure
        self.run_distance_measurement.clear()
        self.led_stopped.set()
        self.motion_controller_stopped.set()
        time.sleep(0.01)

        # and finalize shutting them down
        self.dist_measure_process.join()
        self.led_process.join()
        self.motion_control_process.join()
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

