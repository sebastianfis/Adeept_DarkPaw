from threading import Event
import logging
import time
import serial
from multiprocessing import Process, SimpleQueue
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
init_pwm = {'LF1_init_pwm': 290,
            'LF2_init_pwm': 290,
            'LF3_init_pwm': 270,
            'LB1_init_pwm': 275,
            'LB2_init_pwm': 305,
            'LB3_init_pwm': 310,
            'RF1_init_pwm': 275,
            'RF2_init_pwm': 280,
            'RF3_init_pwm': 295,
            'RB1_init_pwm': 340,
            'RB2_init_pwm': 315,
            'RB3_init_pwm': 310}

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
        message = message + ';\r\n'
        self.serial_port.write(message.encode("utf-8"))
        logger.info('data written to serial:' + message)

    def read_data_from_serial(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode("utf-8").strip()
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
        logging.info('command received:' + command_str)
        if self.last_command != command_str:
            # Note: reset moves are now blocking!
            if command_str == 'S' or command_str == 's' or command_str == 'Stop' or command_str == 'stop':
                self.issue_reset_command()
                self.last_command = None
            elif command_str == 'Dance' or command_str == 'dance':
                # self.issue_reset_command()
                self.last_command = command_str
                self.issue_dance_command(bpm_value=80)
            elif command_str == 'stabilize':
                # self.issue_reset_command()
                self.last_command = command_str
                self.issue_balance_command()
            elif command_str in self.known_gaits and command_str != self.last_command:
                # self.issue_reset_command()
                self.last_command = command_str
                self.issue_walk_command(gait_name=command_str)
            elif command_str in self.known_poses and command_str != self.last_command:
                # self.issue_reset_command()
                self.last_command = command_str
                self.issue_pose_command(pose_name=command_str)
            elif 'velocity_' in command_str:
                value = command_str.split('_')[1]
                self.set_velocity(int(value))
            elif command_str == 'reset_all_actuators':
                self.reset_all_actuators()
            elif 'setpwm_' in command_str:
                temp = command_str.split('_')[1]
                port_no, pwm_value = temp.split(':')
                self.set_init_pwm(port_no, pwm_value)
            else:
                print("I'm sorry! I don't know the command {} :-(\n ".format(command_str) +
                      "I know the following commands: q, stop, dance, \n {0} \n, {1}\n, {2}\n, {3}\n, {4}\n, {5}\n".format(
                          self.known_gaits,
                          self.known_poses,
                          'dance',
                          'stop',
                          'stabilize',
                          'velocity_<value>',
                          'setpwm_<Act_no>:<pwm0_value>',
                          'reset_all_actuators'))
        data = self.read_data_from_serial()
        logging.info(data)


def motion_control_worker(motion_command_queue: SimpleQueue, control_event: Event):
    motion_controller = MotionController()
    while True:
        if control_event.is_set():
            break
        if not motion_command_queue.empty():
            com = motion_command_queue.get()
            if motion_controller.last_command != com:
                motion_controller.execute_command(com)
        data = motion_controller.read_data_from_serial()
        if data is not None:
            logging.info(data)
        time.sleep(0.01)


if __name__ == '__main__':
    test = MotionController()
    test.set_velocity(100)
    test.issue_pose_command()
    motion_command_queue = SimpleQueue()
    motion_controller_stopped = Event()
    motion_controller_stopped.clear()
    motion_control_process = Process(target=motion_control_worker, args=(motion_command_queue,
                                                                         motion_controller_stopped))
    motion_control_process.start()

    while True:
        command = input("Please send a command. I will be happy to follow :-)\n"
                        "type 'quit' to exit \n")
        if command == 'Quit' or command == 'quit':
            motion_command_queue.put('stop')
            time.sleep(2)
            motion_controller_stopped.set()
            time.sleep(0.01)
            motion_control_process.join(timeout=1)
            if motion_control_process.is_alive():
                motion_control_process.terminate()
            break
        else:
            motion_command_queue.put(command)
