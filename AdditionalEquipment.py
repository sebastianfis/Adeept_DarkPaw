import time
import numpy as np
import gpiod
import RPi.GPIO as GPIO
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver
from threading import Event, Thread
import multiprocessing as mp
from multiprocessing import Process, SimpleQueue
from queue import Queue, Empty
import psutil
import os

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
    SPEED_OF_SOUND_CM_PER_S = 34300  # 343 m/s

    def __init__(self, measurement_queue: Queue, control_event: Event,
                 gpio_chip: str = "gpiochip0", GPIO_trigger: int = 23,
                 GPIO_echo: int = 24, cont_measurement_timer: int = 100):
        self.queue = measurement_queue
        self.flag = control_event
        self.period = cont_measurement_timer / 1000.0
        self.trigger_pin = GPIO_trigger
        self.echo_pin = GPIO_echo
        self.chip_name = gpio_chip
        self.last_measurement = 0
        self._timeout = 0.02  # 20 ms max

        # internal state
        self._state = "IDLE"
        self._t_start = None
        self._pulse_start = None

        # gpiod lines
        self.chip = None
        self.trigger_line = None
        self.echo_line = None

    def _setup_gpio(self):
        self.chip = gpiod.Chip(self.chip_name)
        self.trigger_line = self.chip.get_line(self.trigger_pin)
        self.echo_line = self.chip.get_line(self.echo_pin)

        # Configure lines
        self.trigger_line.request(consumer="dist_sensor", type=gpiod.LINE_REQ_DIR_OUT)
        self.trigger_line.set_value(0)
        self.echo_line.request(consumer="dist_sensor", type=gpiod.LINE_REQ_DIR_IN)

    def _cleanup_gpio(self):
        if self.trigger_line:
            self.trigger_line.release()
        if self.echo_line:
            self.echo_line.release()
        if self.chip:
            self.chip.close()

    def take_measurement(self):
        """Non-blocking measurement step"""
        now = time.perf_counter()

        if self._state == "IDLE":
            self.trigger_line.set_value(1)
            self._t_start = now
            self._state = "TRIGGERED"

        elif self._state == "TRIGGERED":
            if now - self._t_start >= 15e-6:  # 15 µs pulse
                self.trigger_line.set_value(0)
                self._state = "WAIT_HIGH"
                self._t_start = now

        elif self._state == "WAIT_HIGH":
            if self.echo_line.get_value():
                self._pulse_start = now
                self._state = "WAIT_LOW"
            elif now - self._t_start > self._timeout:
                self._state = "IDLE"

        elif self._state == "WAIT_LOW":
            if not self.echo_line.get_value():
                pulse_duration = now - self._pulse_start
                distance = (pulse_duration * self.SPEED_OF_SOUND_CM_PER_S) / 2
                if 2 <= distance <= 400:
                    self.last_measurement = distance
                    self.queue.put(distance)
                self._state = "IDLE"
            elif now - self._pulse_start > self._timeout:
                self._state = "IDLE"

    def measure_cont(self):
        """Worker-process loop"""
        self._setup_gpio()
        self.flag.set()
        next_time = time.perf_counter()

        while self.flag.is_set():
            now = time.perf_counter()
            if now >= next_time:
                self.take_measurement()
                next_time = now + self.period
            time.sleep(0.001)  # cooperative sleep

        self._cleanup_gpio()
# class DistSensor:
#     # FIXME: This is probably the reason for faulty dist. measurmeents:
#     def __init__(self, measurement_queue: SimpleQueue, control_event: Event, GPIO_trigger: int = 23, GPIO_echo: int = 24, cont_measurement_timer: int = 100):
#         GPIO.setmode(GPIO.BCM)
#         self.measurement_queue = measurement_queue
#         self.trigger = GPIO_trigger
#         self.echo = GPIO_echo
#         self.last_measurement = 0
#         self.cont_measurement_timer = cont_measurement_timer # in ms
#         self.cont_measurement_flag = control_event
#         self.cont_measurement_flag.clear()
#         # Richtung der GPIO-Pins festlegen (IN / OUT)
#         GPIO.setup(self.trigger, GPIO.OUT)
#         GPIO.setup(self.echo, GPIO.IN, GPIO.PUD_DOWN)
#
#     def take_measurement(self):
#         # setze Trigger auf HIGH
#         GPIO.output(self.trigger, True)
#
#         # setze Trigger nach 0.01ms aus LOW
#         time.sleep(0.00001)
#         GPIO.output(self.trigger, False)
#
#         # speichere Startzeit
#         while GPIO.input(self.echo) == 0:
#             pulse_start = time.time()
#
#         # speichere Ankunftszeit
#         while GPIO.input(self.echo) == 1:
#             pulse_end = time.time()
#
#         # Zeit Differenz zwischen Start und Ankunft
#         pulse_duration = pulse_end - pulse_start
#         # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
#         # und durch 2 teilen, da hin und zurueck
#         distance = (pulse_duration * 34300) / 2
#
#         return distance
#
#     def measure_cont(self):
#         last_exec_time = 0
#         while self.cont_measurement_flag.is_set():
#             now_time = time.perf_counter_ns()
#             if (now_time - last_exec_time) > 1e6 * self.cont_measurement_timer:
#                 self.last_measurement = self.take_measurement()
#                 self.measurement_queue.put(self.last_measurement)
#                 last_exec_time = now_time
# #

class LED:
    def __init__(self, command_queue: SimpleQueue, control_event: Event):
        self.command_queue = command_queue
        self.led_count = 7           # Number of LED pixels.
        # self.led_freq_khz = 800       # LED signal frequency in hertz (usually 800khz)

        # Create NeoPixel object with appropriate configuration.
        self.strip = WS2812SpiDriver(spi_bus=0, spi_device=0, led_count=self.led_count).get_strip()

        self.setColor(0, 0, 0)
        self.strip.show()

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
        self.stopped_flag = control_event
        self.stopped_flag.clear()

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
                    command = self.command_queue.get()
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
        color = [0, 0, 0]
        self.lightMode = 'no_light'
        self.setColor(*color)


def led_worker(command_queue: SimpleQueue, control_event: Event):
    led = LED(command_queue, control_event)
    control_event.clear()
    led.run_lights()


def distance_sensor_worker(distance_queue: Queue, control_event: Event,
                           gpio_chip="gpiochip0", trigger=23, echo=24):
    sensor = DistSensor(distance_queue, control_event,
                        gpio_chip=gpio_chip, GPIO_trigger=trigger,
                        GPIO_echo=echo)
    sensor.measure_cont()


def test_led():
    command_queue = SimpleQueue()
    control_event = Event()
    led_process = Process(target=led_worker, args=(command_queue, control_event))
    led_process.start()

    try:
        while True:
            print('all_good')
            command_queue.put(('all_good', True))  # (mode, breath)
            time.sleep(10)
            print('yellow_alert')
            command_queue.put(('yellow_alert', True))
            time.sleep(10)
            print('red_alert')
            command_queue.put(('red_alert', True))
            time.sleep(10)
            print('remote_controlled')
            command_queue.put(('remote_controlled', True))
            time.sleep(10)
            print('police')
            command_queue.put(('police', False))
            time.sleep(10)
            print('disco')
            command_queue.put(('disco', False))
            time.sleep(10)

    except KeyboardInterrupt:
        print("Exiting...")
        control_event.set()
        led_process.join()
        GPIO.cleanup()


def direct_led_check():
    command_queue = SimpleQueue()
    control_event = Event()
    led = LED(command_queue, control_event)
    led_process = Thread(target=led.run_lights)
    led_process.start()
    try:
        while True:
            print('all_good')
            command_queue.put(('all_good', True))  # (mode, breath)
            time.sleep(10)
            print('yellow_alert')
            command_queue.put(('yellow_alert', True))
            time.sleep(10)
            print('red_alert')
            command_queue.put(('red_alert', True))
            time.sleep(10)
            print('remote_controlled')
            command_queue.put(('remote_controlled', True))
            time.sleep(10)
            print('police')
            command_queue.put(('police', False))
            time.sleep(10)
            print('disco')
            command_queue.put(('disco', False))
            time.sleep(10)

    except KeyboardInterrupt:
        print("Exiting...")
        command_queue.put('exit')
        GPIO.cleanup()


def test_dist_sensor():
    distance_queue = SimpleQueue()
    control_event = Event()
    dist_measure_process = Process(target=distance_sensor_worker, args=(distance_queue, control_event))
    dist_measure_process.start()
    try:
        while True:
            abstand = distance_queue.get()
            print("Gemessene Entfernung = %.1f cm" % abstand)
            time.sleep(0.1)

        # Beim Abbruch durch STRG+C resetten
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        control_event.clear()  # Send stop signal!
        dist_measure_process.join()
        GPIO.cleanup()


if __name__ == '__main__':
    # direct_led_check()
    # test_led()
    test_dist_sensor()
