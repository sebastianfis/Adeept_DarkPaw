from threading import Thread, Event
from queue import Queue
import time
import matplotlib.pyplot as plt
from own_code.SpiderKinematics import RobotModel

# try:
#     import Adafruit_PCA9685
#
#     pwm = Adafruit_PCA9685.PCA9685()
#     pwm.set_pwm_freq(50)
# except:
#     import os
#
#     os.system("sudo pip3 install adafruit-pca9685")
#     import Adafruit_PCA9685
#
#     pwm = Adafruit_PCA9685.PCA9685()
#     pwm.set_pwm_freq(50)

FLB_port = 0
FLM_port = 1
FLE_port = 2

FRB_port = 6
FRM_port = 7
FRE_port = 8

HLB_port = 3
HLM_port = 4
HLE_port = 5

HRB_port = 9
HRM_port = 10
HRE_port = 11

P_port = 12
T_port = 13

FLB_init_pwm = 313
FLM_init_pwm = 305
FLE_init_pwm = 313

FRB_init_pwm = 325
FRM_init_pwm = 281
FRE_init_pwm = 301

HLB_init_pwm = 312
HLM_init_pwm = 287
HLE_init_pwm = 260

HRB_init_pwm = 305
HRM_init_pwm = 195
HRE_init_pwm = 340

P_init_pwm = 300
T_init_pwm = 300

robot_model = RobotModel(FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm,
                         HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm)
q = Queue()


class SequentialImplementation:
    def __init__(self, robot_mdl: RobotModel, pwm_driver):  # Adafruit_PCA9685.PCA9685):
        self.run_time = 7
        self.robot_model = robot_mdl
        self.pwm_driver = pwm_driver
        self.run_flag = True

    def run(self):
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        exec_freq = []
        while self.run_flag:
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) >= 1e9 / self.robot_model.update_freq:
                exec_freq.append(1e9 / (now_time - last_exec_time))
                print('ran for {} seconds'.format((now_time - start_time) / 1e9))
                pass
                #TODO: Implement PWM update from robot gait list here!
                last_exec_time = now_time
            if now_time - start_time >= 1e9 * self.run_time:
                self.run_flag = False

        print('now I am done!')
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(exec_freq)
        ax.set_ylim((0, 25))
        ax.ticklabel_format(useOffset=False, style='plain')
        plt.grid('on')
        plt.show()


class TestClass:
    def __init__(self, robot_mdl: RobotModel, pwm_driver, queue):  # Adafruit_PCA9685.PCA9685):
        self.run_flag = Event()
        self.run_time = 7
        self.robot_model = robot_mdl
        self.pwm_driver = pwm_driver
        self.q = queue

    def worker_function(self):
        start_time = time.perf_counter_ns()
        last_exec_time = start_time
        self.run_flag.set()
        exec_freq = []
        while self.run_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e9/self.robot_model.update_freq:
                timestamp = (now_time - start_time) / 1e9
                exec_freq.append(1e9/(now_time - last_exec_time))
                print('ran for {} seconds'.format(timestamp))
                pass
                # TODO: Implement PWM update from robot gait list here!
                last_exec_time = now_time
        self.q.put(exec_freq)
        print('now I am done!')

    def control_function(self):
        time.sleep(self.run_time)
        self.run_flag.clear()
        print('terminating worker thread after {} seconds'.format(self.run_time))

    def run(self):
        control = Thread(target=self.control_function)
        worker = Thread(target=self.worker_function)
        control.start()
        worker.start()
        control.join()
        worker.join()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(self.q.get())
        ax.set_ylim((0, 25))
        ax.ticklabel_format(useOffset=False, style='plain')
        plt.grid('on')
        plt.show()


if __name__ == '__main__':
    # test = TestClass(robot_mdl=robot_model, pwm_driver=None, queue=q)
    test = SequentialImplementation(robot_mdl=robot_model, pwm_driver=None)
    test.run()
