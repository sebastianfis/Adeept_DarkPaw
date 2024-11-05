# from mpu6050 import mpu6050
import time
import numpy as np
import RPi.GPIO as GPIO
from rpi_ws281x import * #Color, Adafruit_NeoPixel
from threading import Event, Lock

# NOTE: ACC-Sensor input will gor directly into the ESP, so python code is not needed!

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

class DistSensor:
    def __init__(self, GPIO_trigger: int = 18, GPIO_echo: int = 24, cont_measurement_timer: int = 100):
        self.trigger = GPIO_trigger
        self.echo = GPIO_echo
        self.last_measurement = 0
        self.cont_measurement_timer = cont_measurement_timer
        self.cont_measurement_flag = Event()
        self.cont_measurement_flag.clear()
        self.lock = Lock()
        # Richtung der GPIO-Pins festlegen (IN / OUT)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

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

    def function_test(self):
        try:
            while True:
                abstand = self.take_measurement()
                print("Gemessene Entfernung = %.1f cm" % abstand)
                time.sleep(1)

            # Beim Abbruch durch STRG+C resetten
        except KeyboardInterrupt:
            print("Messung vom User gestoppt")
            GPIO.cleanup()

    def measure_cont(self):
        last_exec_time = 0
        while self.cont_measurement_flag.is_set():
            now_time = time.perf_counter_ns()
            if (now_time - last_exec_time) > 1e6 * self.cont_measurement_timer:
                with self.lock:
                    self.last_measurement = self.take_measurement()
                last_exec_time = now_time

    def read_last_measurement(self):
        with self.lock:
            return_value = self.last_measurement
        return return_value


class LED:
    def __init__(self, led_pin: int = 12):
        # Note: This will be highly experimental! Test, if the coral dev board can stomach this!
        self.led_count = 7           # Number of LED pixels.
        self.led_pin = led_pin       # GPIO pin connected to the pixels (18 uses PWM!).
        self.led_freq_hz = 800000    # LED signal frequency in hertz (usually 800khz)
        self.led_dma = 10            # DMA channel to use for generating signal (try 10)
        self.led_brightness = 255    # Set to 0 for darkest and 255 for brightest
        self.led_invert = False      # True to invert the signal (when using NPN transistor level shift)
        self.led_channel = 0         # set to '1' for GPIOs 13, 19, 41, 45 or 53

        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.led_count, self.led_pin, self.led_freq_hz, self.led_dma, self.led_invert,
                                       self.led_brightness, self.led_channel)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

        self.all_good_color = [0, 0, 255]
        self.yellow_alert_color = [255, 100, 0]
        self.red_alert_color = [255, 0, 0]
        self.remote_controlled_color = [0, 255, 0]
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
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()

    def setSomeColor(self, R, G, B, ID):
        color = Color(int(R), int(G), int(B))
        self.strip.setPixelColor(ID, color)
        self.strip.show()

    def policeProcessing(self):
        for i in range(0, 3):
            for j in range(0, self.led_count):
                self.setSomeColor(0, 0, 255, j)
            time.sleep(0.05)
            for j in range(0, self.led_count):
                self.setSomeColor(0, 0, 0, j)
            time.sleep(0.05)

        time.sleep(0.1)
        for i in range(0, 3):
            for j in range(0, self.led_count):
                self.setSomeColor(255, 0, 0, j)
            time.sleep(0.05)
            for j in range(0, self.led_count):
                self.setSomeColor(0, 0, 0, j)
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
        time.sleep(0.5)

    def light_setter(self, set_command: str, breath=False):
        assert set_command in self.known_light_modes
        with self.lock:
            self.breath_flag = breath
            self.lightMode = set_command

    def run_lights(self):
        while not self.stopped_flag.is_set():
            with self.lock:
                breath = self.breath_flag
                set_command = self.lightMode
            if set_command == 'police':
                self.breath_flag =False
                self.policeProcessing()
                continue
            elif set_command == 'disco':
                self.breath_flag = False
                self.discoProcessing()
                continue
            elif set_command == 'all_good':
                color = self.all_good_color
            elif set_command == 'yellow_alert':
                color = self.yellow_alert_color
            elif set_command == 'red_alert':
                color = self.red_alert_color
            elif set_command == 'remote_controlled':
                color = self.remote_controlled_color
            else:
                color = [0, 0, 0]
            if breath:
                self.breathProcessing(*color)
            else:
                self.setColor(*color)
                time.sleep(0.05)
