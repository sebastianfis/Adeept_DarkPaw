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
                    pwm_driver.set_pwm(ports[y + x + str(ii+1)], 0, step_dict[y + x + 'L_PWM'][jj][ii])
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
    def __init__(self, robot_mdl: RobotModel, pwm_driver, queue, run_time=7, gait_name='move_forward'):  # Adafruit_PCA9685.PCA9685):
        self.run_flag = Event()
        self.run_time = run_time
        self.robot_model = robot_mdl
        self.q = queue
        self.current_gait_no = 0
        self.init_gait = Event()
        self.return_gait = Event()
        self.pose_reached = Event()

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
        if self.pwm_driver is not None:
            self.set_init_pwm()

    def set_init_pwm(self):
        for x in ['F','B']:
            for y in ['L', 'R']:
                for ii in [0, 1, 2]:
                    self.pwm_driver.set_pwm(ports[y + x + str(ii+1)], 0, init_pwm[y + x + str(ii+1)])

    def walk_function(self):
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        self.run_flag.set()
        self.init_gait.set()
        exec_freq = []
        ii = 0
        jj = 0
        while self.run_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e9/self.robot_model.update_freq:
                # timestamp = (now_time - start_time) / 1e9
                exec_freq.append(1e9/(now_time - last_exec_time))
                # print('ran for {} seconds'.format(timestamp))
                if self.init_gait.is_set():
                    step = self.robot_model.gaits[self.current_gait_no].init_gait_list[ii]
                    set_pwm_values(step, jj, self.robot_model, self.pwm_driver)
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
                    print('executing step sequence [' + str(ii) + '][' + str(jj) + ']')
                    jj += 1
                    if jj == len(step['LFL_PWM']):
                        jj = 0
                        ii += 1
                    if ii == len(self.robot_model.gaits[self.current_gait_no].gait_list):
                        ii = 0
                last_exec_time = now_time
        self.q.put(exec_freq)
        print('Movement stopped!')

    def test_poses(self):
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        pose_duration = 2
        pose_no = 0
        exec_freq = []
        self.run_flag.set()
        ii = 0
        jj = 0
        pose_list = np.random.permutation(len(self.robot_model.poses))
        pose_movement = self.robot_model.poses[pose_no].calc_pose_dict()
        self.pose_reached.set()
        while self.run_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e9 * pose_duration:
                # pose_no = rng.integers(len(self.robot_model.poses))
                pose_movement = self.robot_model.poses[pose_list[ii]].calc_pose_dict()
                print('Setting pose to {0} after {1} s'.format(self.robot_model.poses[pose_list[ii]].name,
                                                               (now_time - last_exec_time)/1e9))
                self.pose_reached.clear()
                jj = 0
            if (now_time - last_exec_time) > 1e9/self.robot_model.update_freq and not self.pose_reached.is_set():
                exec_freq.append(1e9 / (now_time - last_exec_time))
                set_pwm_values(pose_movement, jj, self.robot_model, self.pwm_driver)
                jj += 1
                last_exec_time = now_time
                if jj == len(pose_movement['LFL_PWM']):
                    jj = 0
                    self.pose_reached.set()
                    ii += 1
                if ii == len(pose_list):
                    ii = 0

        self.q.put(exec_freq)
        print('Posing stopped!')

    def control_function(self, test_poses=False):
        self.run_flag.clear()
        reset_gait = self.robot_model.calc_reset_move()
        print('terminating worker thread after {} seconds.'.format(self.run_time))
        self.return_gait.set()
        exec_freq = self.q.get()
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        ii = 0
        jj = 0
        if not test_poses:
            print('Executing reset step sequence.')
            while self.return_gait.is_set():
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
                        self.return_gait.clear()
                        self.set_init_pwm()
                    last_exec_time = now_time
        else:
            print('Executing pose reset')
            pose_movement = self.robot_model.poses[0].calc_pose_dict()
            while self.return_gait.is_set():
                now_time = time.perf_counter_ns()
                if (now_time - last_exec_time) > 1e9 / self.robot_model.update_freq:
                    timestamp = (now_time - start_time) / 1e9
                    exec_freq.append(1e9 / (now_time - last_exec_time))
                    set_pwm_values(pose_movement, jj, self.robot_model, self.pwm_driver)
                    jj += 1
                    last_exec_time = now_time
                    if jj == len(pose_movement['LFL_PWM']):
                        jj = 0
                        ii += 1
                    if ii == len(pose_movement):
                        self.return_gait.clear()
        self.q.put(exec_freq)
        print('ran for additional {} seconds to reset leg positions to neutral'.format(timestamp))
        print('Total execution time: {} seconds'.format(self.run_time+timestamp))

    def run(self, test_poses=False):
        if test_poses:
            worker_function = self.test_poses
        else:
            worker_function = self.walk_function
        control = Timer(self.run_time, self.control_function, args=[test_poses])
        worker = Thread(target=worker_function)
        control.start()
        worker.start()
        control.join()
        worker.join()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(self.q.get())
        ax.set_ylim((48, 52))
        ax.ticklabel_format(useOffset=False, style='plain')
        plt.grid('on')
        plt.show()


if __name__ == '__main__':
    robot_model.set_velocity(100)
    test = RobotController(robot_mdl=robot_model, run_time=30,
                           pwm_driver=None, queue=q, gait_name='move_forward')
    # test = SequentialImplementation(robot_mdl=robot_model, pwm_driver=None)
    test.run(test_poses=False)
