# from mpu6050 import mpu6050
import time
import numpy as np
import RPi.GPIO as GPIO
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver
from threading import Event, Lock, Thread
from queue import Queue
from multiprocessing import Process
import psutil
import os

# NOTE: ACC-Sensor input will go directly into the ESP, so python code is not needed!

# class AccSensor:
#     def __init__(self):
#         self.sensor = mpu6050(0x68)
#
#     def read_all_sensor_data(self, average_num=3):
#         # Read the accelerometer values
#         accelerometer_data = self.read_accel_data(average_num)
#         # Read the gyroscope values
#         gyroscope_data = self.read_gyro_data(average_num)
#         # Read temp
#         temperature = 0
#         for i in range(0, average_num):
#             temperature += self.sensor.get_temp()
#         temperature /= average_num
#         return accelerometer_data, gyroscope_data, temperature
#
#     def read_accel_data(self, average_num=3):
#         acc_data = {'x': 0, 'y': 0, 'z': 0}
#         for i in range(0, average_num):
#             accelerometer_data = self.read_accel_data()
#             acc_data['x'] += accelerometer_data['x']
#             acc_data['y'] += accelerometer_data['y']
#             acc_data['z'] += accelerometer_data['z']
#         for direction in acc_data.keys():
#             acc_data[direction] /= average_num
#         # due to sensor mounting y and z must be inverted!
#         acc_data['y'] *= -1
#         acc_data['z'] *= -1
#         return acc_data
#
#     def read_gyro_data(self, average_num=3):
#         gyro_data = {'x': 0, 'y': 0, 'z': 0}
#         for i in range(0, average_num):
#             gyroscope_data = self.sensor.get_gyro_data()
#             gyro_data['x'] += gyroscope_data['x']
#             gyro_data['y'] += gyroscope_data['y']
#             gyro_data['z'] += gyroscope_data['z']
#         for direction in gyro_data.keys():
#             gyro_data[direction] /= average_num
#         # due to sensor mounting y and z must be inverted!
#         gyro_data['y'] *= -1
#         gyro_data['z'] *= -1
#         return gyro_data
#
#     def read_body_angles(self):
#         accel_data = self.read_accel_data()
#         theta_x = np.rad2deg(np.arctan2(accel_data['x']/accel_data['z']))
#         theta_y = np.rad2deg(np.arctan2(accel_data['y']/accel_data['z']))
#         return theta_x, theta_y
#
#     def funtion_test(self, average_num=3):
#         try:
#             while True:
#                 print('Taking measurement with averaging over % samples' % average_num)
#                 accelerometer_data, gyroscope_data, temperature = self.read_all_sensor_data(average_num)
#
#                 print('Measured acceleration: X=%.3f m/s², Y=%.3f m/s², Z=%.3f m/s²' % (accelerometer_data['x'],
#                                                                                         accelerometer_data['x'],
#                                                                                         accelerometer_data['x']))
#                 print('Measured angle velocity: X=%.3f deg/s, Y=%.3f deg/s, Z=%.3f deg/s' % (gyroscope_data['x'],
#                                                                                              gyroscope_data['x'],
#                                                                                              gyroscope_data['x']))
#                 print("Measured TEmperatre: %.1f deg C" % temperature)
#                 time.sleep(1)
#
#             # Beim Abbruch durch STRG+C resetten
#         except KeyboardInterrupt:
#             print("Messung vom User gestoppt")
#

GPIO.setmode(GPIO.BCM)


def get_cpu_tempfunc():
    """ Return CPU temperature """
    result = 0
    mypath = "/sys/class/thermal/thermal_zone0/temp"
    with open(mypath, 'r') as mytmpfile:
        for line in mytmpfile:
            result = line

    result = float(result)/1000
    result = round(result, 1)
    return str(result)


def get_gpu_tempfunc():
    """ Return GPU temperature as a character string"""
    res = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline()
    return res.replace("temp=", "")


def get_cpu_use():
    """ Return CPU usage using psutil"""
    cpu_cent = psutil.cpu_percent()
    return str(cpu_cent)


def get_ram_info():
    """ Return RAM usage using psutil """
    ram_cent = psutil.virtual_memory()[2]
    return str(ram_cent)


class DistSensor:
    def __init__(self, measurement_queue: Queue, GPIO_trigger: int = 23, GPIO_echo: int = 24, cont_measurement_timer: int = 100):
        self.measurement_queue = measurement_queue
        self.trigger = GPIO_trigger
        self.echo = GPIO_echo
        self.last_measurement = 0
        self.cont_measurement_timer = cont_measurement_timer # in ms
        self.cont_measurement_flag = Event()
        self.cont_measurement_flag.clear()
        # Richtung der GPIO-Pins festlegen (IN / OUT)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN, GPIO.PUD_DOWN)

    def take_measurement(self):
        # setze Trigger auf HIGH
        GPIO.output(self.trigger, True)

        # setze Trigger nach 0.01ms aus LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        # speichere Startzeit
        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()

        # speichere Ankunftszeit
        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()

        # Zeit Differenz zwischen Start und Ankunft
        pulse_duration = pulse_end - pulse_start
        # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
        # und durch 2 teilen, da hin und zurueck
        distance = (pulse_duration * 34300) / 2

        return distance

    def enable_cont_meaurement(self):
        self.cont_measurement_flag.set()

    def disable_cont_meaurement(self):
        self.cont_measurement_flag.clear()

    def measure_cont(self):
        last_exec_time = 0
        while self.cont_measurement_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e6 * self.cont_measurement_timer:
                self.last_measurement = self.take_measurement()
                last_exec_time = now_time

    def read_last_measurement(self):
        self.measurement_queue.put(self.last_measurement)


class LED:
    def __init__(self, command_queue: Queue):
        self.command_queue = command_queue
        self.led_count = 7           # Number of LED pixels.
        # self.led_freq_khz = 800       # LED signal frequency in hertz (usually 800khz)

        # Create NeoPixel object with appropriate configuration.
        self.strip = WS2812SpiDriver(spi_bus=0, spi_device=0, led_count=self.led_count).get_strip()

        self.all_good_color = (0, 0, 255)
        self.yellow_alert_color = (255, 100, 0)
        self.red_alert_color = (255, 0, 0)
        self.remote_controlled_color = (0, 255, 0)
        self.breathSteps = 20
        self.rng = np.random.default_rng()

        self.lightMode = 'nolight'
        self.known_light_modes = ['nolight', 'police', 'disco', 'all_good',
                                  'yellow_alert', 'red_alert', 'remote_controlled']
        self.breath_flag = False
        self.stopped_flag = Event()
        self.stopped_flag.clear()
        self.lock = Lock()

        GPIO.setwarnings(False)

    # Define functions which animate LEDs in various ways.
    def setColor(self, R, G, B):
        """Wipe color across display a pixel at a time."""
        color = Color(int(R), int(G), int(B))
        self.strip.set_all_pixels(color)
        self.strip.show()

    def setSomeColor(self, R, G, B, ID):
        color = Color(int(R), int(G), int(B))
        self.strip.set_pixel_color(ID, color)

    def policeProcessing(self):
        for i in range(0, 3):
            for j in range(0, self.led_count):
                self.setSomeColor(0, 0, 255, j)
            self.strip.show()
            time.sleep(0.05)
            for j in range(0, self.led_count):
                self.setSomeColor(0, 0, 0, j)
            self.strip.show()
            time.sleep(0.05)

        time.sleep(0.1)
        for i in range(0, 3):
            for j in range(0, self.led_count):
                self.setSomeColor(255, 0, 0, j)
            self.strip.show()
            time.sleep(0.05)
            for j in range(0, self.led_count):
                self.setSomeColor(0, 0, 0, j)
            self.strip.show()
            time.sleep(0.05)
        time.sleep(0.1)

    def breathProcessing(self, R, G, B):
        for i in range(0, self.breathSteps):
            if not self.breath_flag:
                break
            self.setColor(R * i / self.breathSteps, G * i / self.breathSteps, B * i / self.breathSteps)
            time.sleep(0.05)

        for i in range(0, self.breathSteps):
            if not self.breath_flag:
                break
            self.setColor(R - (R * i / self.breathSteps),
                          G - (G * i / self.breathSteps),
                          B - (B * i / self.breathSteps))
            time.sleep(0.05)

    def discoProcessing(self):
        for i in range(0, self.led_count):
            color = [0, 0, 0]
            color_choice = int(np.round(self.rng.random()*2))
            color[color_choice] = self.rng.random()*255
            color_choice = int(np.round(self.rng.random() * 2))
            color[color_choice] = self.rng.random() * 255
            self.setSomeColor(*color, i)
        self.strip.show()
        time.sleep(0.5)

    def run_lights(self):
        while not self.stopped_flag.is_set():
            try:
                # Check if there are new commands
                if not self.command_queue.empty():
                    command = self.command_queue.get_nowait()
                    if command == 'exit':
                        self.setColor(0, 0, 0)
                        self.breath_flag = False
                        self.stopped_flag.set()
                    if isinstance(command, tuple):
                        self.lightMode, self.breath_flag = command

                if self.lightMode == 'police':
                    self.policeProcessing()
                    continue
                elif self.lightMode == 'disco':
                    self.discoProcessing()
                    continue
                elif self.lightMode == 'all_good':
                    color = self.all_good_color
                elif self.lightMode == 'yellow_alert':
                    color = self.yellow_alert_color
                elif self.lightMode == 'red_alert':
                    color = self.red_alert_color
                elif self.lightMode == 'remote_controlled':
                    color = self.remote_controlled_color
                else:
                    color = [0, 0, 0]
                    self.lightMode = 'no_light'
                if self.breath_flag:
                    self.breathProcessing(*color)
                else:
                    self.setColor(*color)
                    time.sleep(0.05)

            except Exception as e:
                print(f"LED process error: {e}")
                break

def test_led():
    command_queue = Queue()
    led_instance = LED(command_queue)
    led_process = Process(target=led_instance.run_lights)
    led_process.start()

    try:
        while True:
            command_queue.put(('all_good', True))  # (mode, breath)
            time.sleep(10)
            command_queue.put(('yellow_alert', True))
            time.sleep(10)
            command_queue.put(('red_alert', True))
            time.sleep(10)
            command_queue.put(('remote_controlled', True))
            time.sleep(10)
            command_queue.put(('police', False))
            time.sleep(10)
            command_queue.put(('disco', False))
            time.sleep(10)

    except KeyboardInterrupt:
        print("Exiting...")
        command_queue.put('exit')
        led_process.join()
        GPIO.cleanup()


def test_dist_sensor():
    dist_sensor = DistSensor()
    dist_sensor.enable_cont_meaurement()
    dist_measure_thread = Thread(target=dist_sensor.measure_cont)
    dist_measure_thread.start()
    try:
        while True:
            abstand = dist_sensor.read_last_measurement()
            print("Gemessene Entfernung = %.1f cm" % abstand)
            time.sleep(0.1)

        # Beim Abbruch durch STRG+C resetten
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        dist_sensor.disable_cont_meaurement()
        dist_measure_thread.join()
        GPIO.cleanup()


if __name__ == '__main__':
    test_led()
    # test_dist_sensor()
