from threading import Thread, Event, Timer
from queue import Queue
import time
import matplotlib.pyplot as plt
from own_code.SpiderKinematics import RobotModel
# import Adafruit_PCA9685
#
# pwm = Adafruit_PCA9685.PCA9685()
# pwm.set_pwm_freq(50)

LF2_port = 1
LF3_port = 2
LF1_port = 0

RF2_port = 7
RF3_port = 8
RF1_port = 6

LB2_port = 4
LB3_port = 5
LB1_port = 3

RB2_port = 10
RB3_port = 11
RB1_port = 9

P_port = 12
T_port = 13

LF2_init_pwm = 300
LF3_init_pwm = 300
LF1_init_pwm = 310

RF2_init_pwm = 300
RF3_init_pwm = 295
RF1_init_pwm = 295

LB2_init_pwm = 325
LB3_init_pwm = 315
LB1_init_pwm = 290

RB2_init_pwm = 285
RB3_init_pwm = 305
RB1_init_pwm = 370

P_init_pwm = 300
T_init_pwm = 300

robot_model = RobotModel(LF2_init_pwm, LF3_init_pwm, LF1_init_pwm, RF2_init_pwm, RF3_init_pwm, RF1_init_pwm,
                         LB2_init_pwm, LB3_init_pwm, LB1_init_pwm, RB2_init_pwm, RB3_init_pwm, RB1_init_pwm)
q = Queue()


def set_pwm_values(step_dict: dict, jj, robot_mdl: RobotModel, pwm_driver):
    """
    function to immediatly set the PWM values of the robot actuators to values passed by a SpiderKinematic Gait class
    object gait list item. index jj denotes the point in the list of tuples, where to take the values from.
    After setting the PWM values, the current position tracking of the SpiderLeg class is updated for each robot leg.
    """
    if pwm_driver is not None:
        pwm_driver.set_pwm(LF1_port, 0, step_dict['LFL_PWM'][jj][0])
        pwm_driver.set_pwm(LF2_port, 0, step_dict['LFL_PWM'][jj][1])
        pwm_driver.set_pwm(LF3_port, 0, step_dict['LFL_PWM'][jj][2])
        pwm_driver.set_pwm(LB1_port, 0, step_dict['LBL_PWM'][jj][0])
        pwm_driver.set_pwm(LB2_port, 0, step_dict['LBL_PWM'][jj][1])
        pwm_driver.set_pwm(LB3_port, 0, step_dict['LBL_PWM'][jj][2])
        pwm_driver.set_pwm(RF1_port, 0, step_dict['RFL_PWM'][jj][0])
        pwm_driver.set_pwm(RF2_port, 0, step_dict['RFL_PWM'][jj][1])
        pwm_driver.set_pwm(RF3_port, 0, step_dict['RFL_PWM'][jj][2])
        pwm_driver.set_pwm(RB1_port, 0, step_dict['RBL_PWM'][jj][0])
        pwm_driver.set_pwm(RB2_port, 0, step_dict['RBL_PWM'][jj][1])
        pwm_driver.set_pwm(RB3_port, 0, step_dict['RBL_PWM'][jj][2])
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
        self.current_gait_no = None
        self.init_gait = Event()
        self.return_gait = Event()

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
            self.pwm_driver.set_pwm(LF1_port, 0, LF1_init_pwm)
            self.pwm_driver.set_pwm(LF2_port, 0, LF2_init_pwm)
            self.pwm_driver.set_pwm(LF3_port, 0, LF3_init_pwm)
            self.pwm_driver.set_pwm(LB1_port, 0, LB1_init_pwm)
            self.pwm_driver.set_pwm(LB2_port, 0, LB2_init_pwm)
            self.pwm_driver.set_pwm(LB3_port, 0, LB3_init_pwm)
            self.pwm_driver.set_pwm(RF1_port, 0, RF1_init_pwm)
            self.pwm_driver.set_pwm(RF2_port, 0, RF2_init_pwm)
            self.pwm_driver.set_pwm(RF3_port, 0, RF3_init_pwm)
            self.pwm_driver.set_pwm(RB1_port, 0, RB1_init_pwm)
            self.pwm_driver.set_pwm(RB2_port, 0, RB2_init_pwm)
            self.pwm_driver.set_pwm(RB3_port, 0, RB3_init_pwm)

    def walk_function(self, walk_command: str):
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

    def control_function(self):
        self.run_flag.clear()
        reset_gait = self.robot_model.calc_reset_move()
        print('terminating worker thread after {} seconds. Executing reset step sequence.'.format(self.run_time))
        self.return_gait.set()
        exec_freq = self.q.get()
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        ii = 0
        jj = 0
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
                last_exec_time = now_time
        print('ran for additional {} seconds to reset leg positions to neutral'.format(timestamp))
        self.q.put(exec_freq)
        print('Total execution time: {} seconds'.format(self.run_time+timestamp))

    def test_poses(self):
        # TODO: implement test code for poses
        pass

    def run(self, test_poses=False):
        if test_poses:
            worker_function = self.test_poses
        else:
            worker_function = self.walk_function
        control = Timer(self.run_time, self.control_function)
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
    test = RobotController(robot_mdl=robot_model, pwm_driver=None, queue=q, gait_name='move_forward')
    # test = SequentialImplementation(robot_mdl=robot_model, pwm_driver=None)
    test.run()

    # TODO: Add function calls to test poses
