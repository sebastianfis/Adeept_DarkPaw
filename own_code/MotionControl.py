from threading import Event
import logging
# from queue import Queue
import time
# import numpy as np
# import matplotlib.pyplot as plt
import serial
# from SpiderKinematics import RobotModel

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


class MotionController:
    def __init__(self):
        self.run_flag = Event()
        self.current_gait_no = None
        self.current_pose_no = None
        self.current_gait_name = None
        self.current_pose_name = None
        self.last_command = None
        self.known_gaits = ['move_forward', 'move_backward', 'move_right', 'move_left', 'turn_right', 'turn_left']
        self.gait_commands = ['gmf', 'gmb', 'gmr', 'gml', 'gtr', 'gtl']
        self.known_poses = ['neutral', 'look_up', 'look_down', 'lean_right', 'lean_left', 'high', 'low']
        self.pose_commands = ['pn', 'plu', 'pld', 'plr', 'pll', 'phi', 'plo']
        self.serial_port = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, timeout=0.05)

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

    def execute_command(self, command_str: str):
        data = self.read_data_from_serial()
        logging.info(data)
        logging.info('command received:' + command_str)
        if self.last_command != command_str:
            # Note: reset moves are now blocking!
            if command_str == 'S' or command_str == 's' or command_str == 'Stop' or command_str == 'stop':
                self.issue_reset_command()
                self.last_command = None
            elif command_str == 'Dance' or command_str == 'dance':
                self.issue_reset_command()
                self.last_command = command_str
                self.issue_dance_command(bpm_value=80)
            elif command_str in self.known_gaits and command_str != self.last_command:
                self.issue_reset_command()
                self.last_command = command_str
                self.issue_walk_command(gait_name=command_str)
            elif command_str in self.known_poses and command_str != self.last_command:
                self.issue_reset_command()
                self.last_command = command_str
                self.issue_pose_command(pose_name=command_str)
            else:
                print("I'm sorry! I don't know the command {} :-(\n ".format(command_str) +
                      "I know the following commands: q, stop, dance, \n {0} \n, {1}\n, {2}\n, {3}\n".format(
                          self.known_gaits,
                          self.known_poses,
                          'dance',
                          'stop'))


if __name__ == '__main__':
    test = MotionController()
    test.set_velocity(100)
    test.issue_pose_command()

    while True:
        command = input("Please send a command. I will be happy to follow :-)\n"
                        "type 'quit' to exit \n")
        if command == 'Quit' or command == 'quit':
            test.issue_reset_command()
            test.last_command = None
            break
        else:
            test.execute_command(command)
