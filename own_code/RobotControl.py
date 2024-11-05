from threading import Thread, Event, Timer
from queue import Queue
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
# from SpiderKinematics import RobotModel

run_on_Raspi = True

if run_on_Raspi:
    from AdditionalEquipment import LED, DistSensor
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    led = LED()
    distance_sensor = DistSensor()
else:
    led = None
    distance_sensor = None


translation_table = {'w': 'move_forward',
                     'x': 'move_backward',
                     'a': 'move_left',
                     'd': 'move_right',
                     'q': 'turn_left',
                     'e': 'turn_right'}

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

# Create actuator direction dictionary:
act_dir = {'LF1_act_dir': -1,
           'LF2_act_dir': 1,
           'LF3_act_dir': -1,
           'LB1_act_dir': -1,
           'LB2_act_dir': -1,
           'LB3_act_dir': 1,
           'RF1_act_dir': 1,
           'RF2_act_dir': -1,
           'RF3_act_dir': 1,
           'RB1_act_dir': 1,
           'RB2_act_dir': 1,
           'RB3_act_dir': -1}

debug = False

# TODO: Add Dist sensor code and evaluation

class RobotController:
    def __init__(self, led_control, dist_sensor):
        self.run_flag = Event()
        self.current_gait_no = None
        self.current_pose_no = None
        self.current_gait_name = None
        self.current_pose_name = None
        self.last_command = None
        self.known_gaits = ['move_forward', 'move_backward', 'move_right', 'move_left', 'turn-right', 'turn-left']
        self.gait_commands = ['gmf', 'gmb', 'gmr', 'gml', 'gtr', 'gtl']
        self.known_poses = ['neutral', 'look_up', 'look_down', 'lean_right', 'lean_left', 'high', 'low']
        self.pose_commands = ['pn', 'plu', 'pld', 'plr', 'pll', 'phi', 'plo']
        self.led = led_control
        self.dist_sensor = dist_sensor
        self.serial_port = serial.Serial(port='COM4', baudrate=115200, timeout=0.05)

    def write_data_to_serial(self, message: str):
        self.serial_port.write(bytes(message + ';', 'utf-8'))

    def read_data_from_serial(self):
        data = self.serial_port.readline()
        return data

    def set_init_pwm(self, port_no: int, pwm_value: int):
        self.write_data_to_serial('cp' + str(port_no) + ',' + str(pwm_value))

    def set_actuator_direction(self, port_no: int, pwm_value: int):
        self.write_data_to_serial('cd' + str(port_no) + ',' + str(pwm_value))

    def reset_all_actuators(self):
        for letter1 in ['L', 'R']:
            for letter2 in ['F', 'B']:
                for letter3 in ['1', '2', '3']:
                    self.set_init_pwm(ports[letter1 + letter2 + letter3 + '_port'],
                                      init_pwm[letter1 + letter2 + letter3 + '_init_pwm'])
                    self.set_actuator_direction(ports[letter1 + letter2 + letter3 + '_port'],
                                                act_dir[letter1 + letter2 + letter3 + '_act_dir'])

    def set_velocity(self, velocity: int = 100):
        if self.current_gait_no is not None:
            self.issue_reset_command()
        self.write_data_to_serial('v' + str(velocity))

    def issue_walk_command(self, gait_name):
        self.current_pose_name = None
        self.current_pose_no = None
        self.current_gait_no = 0

        for gait in self.known_gaits:
            if gait_name == gait:
                self.current_gait_name = gait_name
                break
            else:
                self.current_gait_name = None
                self.current_gait_no += 1
        assert self.current_gait_name is not None, "given gait_name not implemented! Allowed gait_names are: " \
                                                   "'move_forward', 'move_backward', 'move_right', 'move_left', " \
                                                   "'turn_right', 'turn_left'"
        self.run_flag.set()
        self.write_data_to_serial(self.gait_commands[self.current_gait_no])

    def issue_pose_command(self, pose_name='neutral'):
        self.current_gait_name = None
        self.current_gait_no = None
        self.current_pose_no = 0
        for pose in self.known_poses:
            if pose == pose_name:
                self.current_pose_name = pose_name
                break
            else:
                self.current_pose_name = None
                self.current_pose_no += 1
        assert self.current_pose_name is not None, "given pose_name not implemented! Allowed pose_names are: " \
                                                   "'neutral', 'look_up', 'look_down', 'lean_right', 'lean_left'," \
                                                   "'high', 'low'"
        if self.run_flag.is_set():
            self.run_flag.clear()
        self.write_data_to_serial(self.pose_commands[self.current_pose_no])

    def issue_dance_command(self, bpm_value: int = 80):
        self.run_flag.set()
        self.write_data_to_serial('d' + str(bpm_value))

    def issue_balance_command(self):
        if self.run_flag.is_set():
            self.run_flag.clear()
        self.write_data_to_serial('b')

    def issue_reset_command(self):
        self.run_flag.clear()
        self.current_gait_name = None
        self.current_gait_no = None
        self.current_pose_name = None
        self.current_pose_no = None
        self.write_data_to_serial('s')

    def run(self):
        worker = Thread(target=self.issue_pose_command)
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
                self.issue_reset_command()
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
                self.issue_reset_command()
                self.last_command = None
                worker.join()
            elif command == 'Dance' or command == 'dance':
                self.issue_reset_command()
                self.last_command = command
                self.led.light_setter('disco')
                worker = Thread(target=self.issue_dance_command)
                worker.start()
            elif command in self.known_gaits and command != self.last_command:
                self.issue_reset_command()
                self.last_command = command
                worker.join()
                worker = Thread(target=self.issue_walk_command, args=[command])
                worker.start()
            elif command in self.known_poses and command != self.last_command:
                self.issue_reset_command()
                self.last_command = command
                worker.join()
                worker = Thread(target=self.issue_pose_command, args=[command])
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
                      "I know the foll0wing commands: q, stop, dance, \n {0} \n, {1}\n, {2}\n".format(self.known_gaits,
                                                                                                     self.known_poses,
                                                                                                     known_light_modes))
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        # exec_freq = self.load_old_freq_data()
        # ax.plot(exec_freq)
        # ax.set_ylim((48, 52))
        # ax.ticklabel_format(useOffset=False, style='plain')
        # plt.grid('on')
        # plt.show()


if __name__ == '__main__':
    test = RobotController(led_control=led, dist_sensor=distance_sensor)
    test.set_velocity(100)
    test.run()
