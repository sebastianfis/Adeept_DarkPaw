from threading import Thread, Event, Timer
from queue import Queue
import time
import numpy as np
import matplotlib.pyplot as plt
from own_code.SpiderKinematics import RobotModel
# import Adafruit_PCA9685
#
# pwm = Adafruit_PCA9685.PCA9685()
# pwm.set_pwm_freq(50)

translation_table = {'w': 'move_forward',
                     'x': 'move_backward',
                     'a': 'move_left',
                     'd': 'move_right',
                     'q': 'turn_left',
                     'e': 'turn_right'}

debug = True

# Create port dictionary:
ports = {'LF1_port': 0, 
         'LF2_port': 1, 
         'LF3_port': 2,
         'LB1_port': 3,
         'LB2_port': 4,
         'LB3_port': 5,
         'RF1_port': 6,
         'RF2_port': 7,
         'RF3_port': 8,
         'RB1_port': 9,
         'RB2_port': 10,
         'RB3_port': 11}

# Create init_pwm dictionary:
init_pwm = {'LF1_init_pwm': 305,
            'LF2_init_pwm': 325,
            'LF3_init_pwm': 330,
            'LB1_init_pwm': 295,
            'LB2_init_pwm': 300,
            'LB3_init_pwm': 285,
            'RF1_init_pwm': 295,
            'RF2_init_pwm': 285,
            'RF3_init_pwm': 290,
            'RB1_init_pwm': 365,
            'RB2_init_pwm': 340,
            'RB3_init_pwm': 345}


robot_model = RobotModel(init_pwm)
q = Queue()
rng = np.random.default_rng()


def set_pwm_values(step_dict: dict, jj, robot_mdl: RobotModel, pwm_driver):
    """
    function to immediatly set the PWM values of the robot actuators to values passed by a SpiderKinematic Gait class
    object gait list item. index jj denotes the point in the list of tuples, where to take the values from.
    After setting the PWM values, the current position tracking of the SpiderLeg class is updated for each robot leg.
    """
    if pwm_driver is not None:
        for x in ['F','B']:
            for y in ['L', 'R']:
                for ii in [0, 1, 2]:
                    pwm_driver.set_pwm(ports[y + x + str(ii+1) + '_port'], 0, step_dict[y + x + 'L_PWM'][jj][ii])
    robot_mdl.left_forward_leg.update_cur_phi(*step_dict['LFL'][jj])
    robot_mdl.left_backward_leg.update_cur_phi(*step_dict['LBL'][jj])
    robot_mdl.right_forward_leg.update_cur_phi(*step_dict['RFL'][jj])
    robot_mdl.right_backward_leg.update_cur_phi(*step_dict['RBL'][jj])


class SequentialControl:
    """
    Note: This sequential implementation is intended to be the base for a later C++ imlementation
    """
    def __init__(self, robot_mdl: RobotModel, pwm_driver, gait_name='move_forward'):  # Adafruit_PCA9685.PCA9685):
        self.run_time = 7
        self.robot_model = robot_mdl
        self.current_gait_no = 0
        for gait in robot_mdl.gaits:
            if gait.name == gait_name:
                self.current_gait_name = gait_name
                break
            else:
                self.current_gait_name = None
                self.current_gait_no += 1
        assert self.current_gait_name is not None, "given gait_name not implemented! Allowed gait_names are: " \
                                                   "'move_forward', 'move_backward', 'move_right', 'move left', " \
                                                   "'turn_right', 'turn_left'"
        self.pwm_driver = pwm_driver
        # self.pwm_driver.set_pwm(LF1_port, 0, LF1_init_pwm)
        # self.pwm_driver.set_pwm(LF2_port, 0, LF2_init_pwm)
        # self.pwm_driver.set_pwm(LF3_port, 0, LF3_init_pwm)
        # self.pwm_driver.set_pwm(LB1_port, 0, LB1_init_pwm)
        # self.pwm_driver.set_pwm(LB2_port, 0, LB2_init_pwm)
        # self.pwm_driver.set_pwm(LB3_port, 0, LB3_init_pwm)
        # self.pwm_driver.set_pwm(RF1_port, 0, RF1_init_pwm)
        # self.pwm_driver.set_pwm(RF2_port, 0, RF2_init_pwm)
        # self.pwm_driver.set_pwm(RF3_port, 0, RF3_init_pwm)
        # self.pwm_driver.set_pwm(RB1_port, 0, RB1_init_pwm)
        # self.pwm_driver.set_pwm(RB2_port, 0, RB2_init_pwm)
        # self.pwm_driver.set_pwm(RB3_port, 0, RB3_init_pwm)
        self.run_flag = True
        self.init_gait = True
        self.return_flag = False
        self.reset_n = 12

    def run(self):
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        exec_freq = []
        ii = 0
        jj = 0
        while self.run_flag:
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) >= 1e9 / self.robot_model.update_freq:
                exec_freq.append(1e9 / (now_time - last_exec_time))
                if self.init_gait:
                    step = self.robot_model.gaits[self.current_gait_no].init_gait_list[ii]
                    set_pwm_values(step, jj, self.robot_model, self.pwm_driver)
                    print('executing init step sequence [' + str(ii) + '][' + str(jj) + ']')
                    jj += 1
                    if jj == len(step['LFL_PWM']):
                        jj = 0
                        ii += 1
                    if ii == len(self.robot_model.gaits[self.current_gait_no].init_gait_list):
                        ii = 0
                        self.init_gait = False
                else:
                    step = self.robot_model.gaits[self.current_gait_no].gait_list[ii]
                    set_pwm_values(step, jj, self.robot_model, self.pwm_driver)
                    print('executing step sequence [' + str(ii) + '][' + str(jj) + ']')
                    jj += 1
                    if jj == len(step['LFL_PWM']):
                        jj = 0
                        ii += 1
                    if ii == len(self.robot_model.gaits[self.current_gait_no].gait_list):
                        ii = 0

                # print('ran for {} seconds'.format((now_time - start_time) / 1e9))

                last_exec_time = now_time
            if now_time - start_time >= 1e9 * self.run_time:
                print('movement timeout! Executing return step sequence')
                last_exec_time = now_time
                ii = 0
                jj = 0
                self.run_flag = False

        print('Movement stopped!')
        reset_gait = self.robot_model.calc_reset_move()
        print('Movement terminated after {} seconds. Executing reset step sequence.'.format(self.run_time))
        self.return_flag = True
        ii = 0
        jj = 0
        while self.return_flag:
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
                    self.return_flag = False
                last_exec_time = now_time
        print('ran for additional {} seconds to reset leg positions to neutral'.format(timestamp-self.run_time))
        print('Total execution time: {} seconds'.format(timestamp))

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(exec_freq)
        ax.set_ylim((0, 25))
        ax.ticklabel_format(useOffset=False, style='plain')
        plt.grid('on')
        plt.show()


class RobotController:
    def __init__(self, robot_mdl: RobotModel, pwm_driver, queue):  # Adafruit_PCA9685.PCA9685):
        self.run_flag = Event()
        self.robot_model = robot_mdl
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

        while True:
            command = input("Please send a command. I will be happy to follow :-)\n"
                         "type 'quit' to exit \n")
            # Note: reset moves are now blocking.
            if command == 'Quit' or command == 'quit':
                self.reset()
                self.last_command = None
                worker.join()
                break
            elif command == 'S' or command == 's' or command == 'Stop' or command == 'stop':
                self.reset()
                self.last_command = None
                worker.join()
            elif command == 'Dance' or command == 'dance':
                self.reset()
                self.last_command = command
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
            else:
                print("I'm sorry! I don't know this command :-(\n "
                      "I know the follwing commands: q, stop, dance, \n {0} \n, {1}\n".format(self.known_gaits,
                                                                                              self.known_poses))
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
    test = RobotController(robot_mdl=robot_model, pwm_driver=None, queue=q)
    test.run()
