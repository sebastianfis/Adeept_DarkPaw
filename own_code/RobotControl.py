from threading import Thread, Event, Timer
from queue import Queue
import time
import numpy as np
import matplotlib.pyplot as plt
# from SpiderKinematics import RobotModel

run_on_Raspi = True

if run_on_Raspi:
    from AdditionalEquipment import LED, DistSensor
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    led = LED()
else:
    led = None


translation_table = {'w': 'move_forward',
                     'x': 'move_backward',
                     'a': 'move_left',
                     'd': 'move_right',
                     'q': 'turn_left',
                     'e': 'turn_right'}

debug = False


q = Queue()

# TODO: Add serial communication, add Dist sensor code and evaluation

class RobotController:
    def __init__(self, led_control, queue):
        self.run_flag = Event()

        self.q = queue
        self.current_gait_no = None
        self.current_pose_no = None
        self.current_gait_name = None
        self.current_pose_name = None
        self.init_gait = Event()
        self.reset_flag = Event()
        self.pose_reached = Event()
        self.last_command = None
        self.known_gaits = []
        self.known_poses = []
        for gait in self.robot_model.gaits:
            self.known_gaits.append(gait.name)
        for pose in self.robot_model.poses:
            self.known_poses.append(pose.name)

        self.pwm_driver = pwm_driver
        if self.pwm_driver is not None:
            self.set_init_pwm()

        self.led = led_control
        self.mpu = mpu_6050interface

    def load_old_freq_data(self):
        exec_freq = []
        while not self.q.empty():
            old_data = self.q.get()
            for item in old_data:
                exec_freq.append(item)
        return exec_freq

    def set_init_pwm(self):
        for x in ['F', 'B']:
            for y in ['L', 'R']:
                for ii in [0, 1, 2]:
                    self.pwm_driver.set_pwm(ports[y + x + str(ii+1) + '_port'], 0,
                                            init_pwm[y + x + str(ii+1) + '_init_pwm'])

    def walk_function(self, gait_name):
        self.current_pose_name = None
        self.current_pose_no = 0
        self.current_gait_no = 0

        for gait in self.robot_model.gaits:
            if gait.name == gait_name:
                self.current_gait_name = gait_name
                break
            else:
                self.current_gait_name = None
                self.current_gait_no += 1
        assert self.current_gait_name is not None, "given gait_name not implemented! Allowed gait_names are: " \
                                                   "'move_forward', 'move_backward', 'move_right', 'move left', " \
                                                   "'turn_right', 'turn_left'"
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        self.run_flag.set()
        self.init_gait.set()
        exec_freq = self.load_old_freq_data()
        ii = 0
        jj = 0
        while self.run_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e9/self.robot_model.update_freq:
                exec_freq.append(1e9/(now_time - last_exec_time))
                if self.init_gait.is_set():
                    step = self.robot_model.gaits[self.current_gait_no].init_gait_list[ii]
                    set_pwm_values(step, jj, self.robot_model, self.pwm_driver)
                    if debug:
                        print('executing init step sequence [' + str(ii) + '][' + str(jj) + ']')
                    jj += 1
                    if jj == len(step['LFL_PWM']):
                        jj = 0
                        ii += 1
                    if ii == len(self.robot_model.gaits[self.current_gait_no].init_gait_list):
                        ii = 0
                        self.init_gait.clear()
                else:
                    step = self.robot_model.gaits[self.current_gait_no].gait_list[ii]
                    set_pwm_values(step, jj, self.robot_model, self.pwm_driver)
                    if debug:
                        print('executing step sequence [' + str(ii) + '][' + str(jj) + ']')
                    jj += 1
                    if jj == len(step['LFL_PWM']):
                        jj = 0
                        ii += 1
                    if ii == len(self.robot_model.gaits[self.current_gait_no].gait_list):
                        ii = 0
                last_exec_time = now_time
        self.q.put(exec_freq)
        if debug:
            print('Movement stopped!')

    def set_pose(self, pose_name='neutral'):
        self.current_gait_name = None
        self.current_gait_no = None
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        self.current_pose_no = 0
        for pose in self.robot_model.poses:
            if pose.name == pose_name:
                self.current_pose_name = pose_name
                break
            else:
                self.current_pose_name = None
                self.current_pose_no += 1
        assert self.current_pose_name is not None, "given pose_name not implemented! Allowed pose_names are: " \
                                                   "'neutral', 'look_up', 'look_down', 'lean_right', 'lean_left'," \
                                                   "'high', 'low', 'lift_LFL', 'lift_LBL', 'lift_RFL', 'lift_RBL'"
        exec_freq = self.load_old_freq_data()
        pose_movement = self.robot_model.poses[self.current_pose_no].calc_pose_dict()
        self.pose_reached.clear()
        if debug:
            print('Setting pose to {0}\n'.format(self.current_pose_name))
        jj = 0
        while not self.pose_reached.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e9/self.robot_model.update_freq and not self.pose_reached.is_set():
                exec_freq.append(1e9 / (now_time - last_exec_time))
                set_pwm_values(pose_movement, jj, self.robot_model, self.pwm_driver)
                jj += 1
                last_exec_time = now_time
                if jj == len(pose_movement['LFL_PWM']):
                    self.pose_reached.set()

        self.q.put(exec_freq)
        if debug:
            print('Pose {0} reached!\n'.format(self.current_pose_name))

    def dance(self, dance_move_duration=0.5):

        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        self.run_flag.set()
        ii = 0
        pose_list = np.random.permutation(len(self.robot_model.poses))
        self.pose_reached.set()
        while self.run_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e9 * dance_move_duration:
                self.set_pose(pose_name=self.robot_model.poses[ii].name)
                ii += 1
                last_exec_time = now_time
                if ii == len(pose_list):
                    ii = 0
        print('Dancing stopped!')

    def reset(self):
        self.run_flag.clear()
        start_time = time.perf_counter_ns()
        timestamp = 0
        ii = 0
        jj = 0
        exec_freq = self.load_old_freq_data()
        if self.last_command is not None:
            self.reset_flag.set()
            if self.last_command in self.known_gaits:
                reset_gait = self.robot_model.calc_reset_move()
                if debug:
                    print('Executing reset step sequence.\n')
                last_exec_time = time.perf_counter_ns()
                while self.reset_flag.is_set():
                    now_time = time.perf_counter_ns()
                    if (now_time - last_exec_time) > 1e9 / self.robot_model.update_freq:
                        timestamp = (now_time - start_time) / 1e9
                        exec_freq.append(1e9 / (now_time - last_exec_time))
                        step = reset_gait[ii]
                        set_pwm_values(step, jj, self.robot_model, self.pwm_driver)
                        print('executing reset step sequence [' + str(ii) + '][' + str(jj) + ']')
                        jj += 1
                        if jj == len(step['LFL_PWM']):
                            jj = 0
                            ii += 1
                        if ii == len(reset_gait):
                            ii = 0
                            self.reset_flag.clear()
                            if self.pwm_driver is not None:
                                self.set_init_pwm()
                        last_exec_time = now_time
                print('ran for additional {} seconds to reset leg positions to neutral\n'.format(timestamp))
            elif self.last_command in self.known_poses or self.last_command == 'Dance' or self.last_command == 'dance':
                if debug:
                    print('Executing pose reset\n')
                self.set_pose(pose_name='neutral')
                self.reset_flag.clear()
            else:
                raise RuntimeError('Resetting to initial state seems to be impossible, as current state is unknown!')
        self.q.put(exec_freq)

    def run(self):
        worker = Thread(target=self.set_pose)
        worker.start()
        if run_on_Raspi:
            lights_thread = Thread(target=self.led.run_lights)
            lights_thread.start()
            known_light_modes = self.led.known_light_modes
        else:
            known_light_modes = ['none', 'police', 'all_good', 'yellow_alert', 'red_alert', 'remote_controlled', 'disco']

        while True:
            command = input("Please send a command. I will be happy to follow :-)\n"
                         "type 'quit' to exit \n")
            # Note: reset moves are now blocking.
            if command == 'Quit' or command == 'quit':
                self.reset()
                self.last_command = None
                worker.join()
                if run_on_Raspi:
                    self.led.light_setter('nolight')
                    time.sleep(0.05)
                    self.led.stopped_flag.set()
                    time.sleep(0.05)
                    lights_thread.join()
                break
            elif command == 'S' or command == 's' or command == 'Stop' or command == 'stop':
                self.reset()
                self.last_command = None
                worker.join()
            elif command == 'Dance' or command == 'dance':
                self.reset()
                self.last_command = command
                self.led.light_setter('disco')
                worker = Thread(target=self.dance)
                worker.start()
            elif command in self.known_gaits and command != self.last_command:
                self.reset()
                self.last_command = command
                worker.join()
                worker = Thread(target=self.walk_function, args=[command])
                worker.start()
            elif command in self.known_poses and command != self.last_command:
                self.reset()
                self.last_command = command
                worker.join()
                worker = Thread(target=self.set_pose, args=[command])
                worker.start()
            elif command in known_light_modes:
                if run_on_Raspi:
                    if command == 'nolight':
                        self.led.light_setter(command, breath=False)
                    else:
                        self.led.light_setter(command, breath=True)
                print("Light mode set to {}".format(command))
            else:
                print("I'm sorry! I don't know the command {} :-(\n ".format(command) +
                      "I know the follwing commands: q, stop, dance, \n {0} \n, {1}\n, {2}\n".format(self.known_gaits,
                                                                                                     self.known_poses,
                                                                                                     known_light_modes))
        fig = plt.figure()
        ax = fig.add_subplot(111)
        exec_freq = self.load_old_freq_data()
        ax.plot(exec_freq)
        ax.set_ylim((48, 52))
        ax.ticklabel_format(useOffset=False, style='plain')
        plt.grid('on')
        plt.show()


if __name__ == '__main__':
    robot_model.set_velocity(100)
    test = RobotController(robot_mdl=robot_model, pwm_driver=pwm, led_control=led, mpu_6050interface=mpu6050, queue=q)
    test.run()
