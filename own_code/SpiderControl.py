import time
import threading
from own_code.SpiderKinematics import RobotModel

try:
    import Adafruit_PCA9685
except ImportError:
    import os
    os.system("sudo pip3 install adafruit-pca9685")
    import Adafruit_PCA9685

try:
    from mpu6050 import mpu6050
    from server import PID
    from server import Kalman_filter
except:
    print('mpu related libraries could not be loaded and the related functions are unavailable.')


class RobotControl:
    def __init__(self, robot_model: RobotModel):
        self.robot_model = robot_model
        self.mode = 'None'
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)
        self.current_walk_still_init = False
        self.ii = 0
        self.jj = 0

        self.FLB_port = 0
        self.FLM_port = 1
        self.FLE_port = 2

        self.FRB_port = 6
        self.FRM_port = 7
        self.FRE_port = 8

        self.HLB_port = 3
        self.HLM_port = 4
        self.HLE_port = 5

        self.HRB_port = 9
        self.HRM_port = 10
        self.HRE_port = 11

        try:
            self.mpu = mpu6050(0x68)
            kp = 0.3
            ki = 0.1
            kd = 0
            self.x_pid = PID.PID(kp,kd,ki)
            self.y_pid = PID.PID(kp,kd,ki)
            self.kalman_filter = Kalman_filter.Kalman_filter(0.001, 0.1)
            print('mpu6050 connected\nmpu6050 is connected and related functions are available.')
        except:
            self.mpu = None
            print('mpu6050 disconnected\nmpu6050 is not connected and the related functions are unavailable.')

    def run_control(self):
        while True:
            if self.mode == 'steady' and self.mpu is not None:
                self.steady()
            elif self.mode == 'turnleft':
                pass
            elif self.mode == 'turnright':
                pass
            elif self.mode == 'walkright':
                pass
            elif self.mode == 'walkleft':
                pass
            elif self.mode == 'forward':
                pass
            elif self.mode == 'backward':
                pass
            else:
                pass

    def steady(self):
        try:
            accelerometer_data = self.mpu.get_accel_data()
            x = accelerometer_data['x']/accelerometer_data['z']
            x_error = self.kalman_filter.kalman(x)
            y = accelerometer_data['y']/accelerometer_data['z']
            y_error = self.kalman_filter.kalman(y)

            if abs(x_error) > 0 or abs(y_error) > 0:
                status_GenOut(0, self.x_pid.GenOut(error=x_error), self.y_pid.GenOut(error=y_error))  #FIXME: Replace function
                direct_M_move()                             #FIXME: Replace function

        except:
            time.sleep(0.1)
            self.mpu = mpu6050(0x68)
            pass



    def walk(self, direction: str, ii=0):
        if direction == 'forward':
            gait = self.robot_model.move_forward
        elif direction == 'backward':
            gait = self.robot_model.move_backward
        elif direction == 'turnleft':
            gait = self.robot_model.turn_counterclockwise
        elif direction == 'turnright':
            gait = self.robot_model.turn_clockwise
        elif direction == 'left':
            gait = self.robot_model.move_left
        elif direction == 'right':
            gait = self.robot_model.move_right
        else:
            raise RuntimeError('gait command {} unknown'.format(direction))

        while ii < len(gait.init_gait_list):
            pass
            # TODO: finish gait control implementation



