import time
import numpy as np

import RPi.GPIO as GPIO
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver
from threading import Event, Thread
from multiprocessing.connection import Connection
from multiprocessing import Process, Pipe
from multiprocessing import Event
import psutil
import os
from collections import deque
import gpiod
from gpiod.line import Direction, Edge, Value


"""
parent_conn, child_conn = Pipe(duplex=False)

    parent_conn → sender

    child_conn → receiver
"""


def recv_latest(conn: Connection):
    data = None
    while conn.poll():
        data = conn.recv()
    return data


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
    def __init__(
        self,
        measurement_connector: Connection,
        control_event: Event,
        gpio_chip: str = "/dev/gpiochip0",
        GPIO_trigger: int = 23,
        GPIO_echo: int = 24,
        cont_measurement_timer: int = 100  # ms
    ):
        self.connector = measurement_connector
        self.flag = control_event
        self.period = cont_measurement_timer / 1000.0

        self.chip_name = gpio_chip
        self.trigger_pin = GPIO_trigger
        self.echo_pin = GPIO_echo
        self.TIMEOUT_S = 1
        self.SPEED_OF_SOUND_CM_PER_S = 34300  # cm/s

        self.raw_history = deque(maxlen=5)
        self.ema_value = None
        self.ema_alpha = 0.3

        self.request = None

    # -------------------------------------------------
    # GPIO setup / cleanup (inside process)
    # -------------------------------------------------
    def _setup_gpio(self):
        chip = gpiod.Chip(self.chip_name)

        config = {
            self.trigger_pin: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE
            ),
            self.echo_pin: gpiod.LineSettings(
                direction=Direction.INPUT,
                edge_detection=Edge.BOTH
            ),
        }

        self.request = chip.request_lines(
            config=config,
            consumer="dist_sensor"
        )

    def _cleanup_gpio(self):
        if self.request:
            self.request.release()
            self.request = None

    # -------------------------------------------------
    # Single blocking measurement (edge-based)
    # -------------------------------------------------
    def take_measurement(self):
        # Flush any stale events
        while self.request.wait_edge_events(0):
            self.request.read_edge_events()

        # Trigger pulse for 15 µs
        self.request.set_value(self.trigger_pin, 1)
        time.sleep(15e-6)
        self.request.set_value(self.trigger_pin, 0)

        deadline = time.monotonic() + self.TIMEOUT_S

        # Wait for a rising edge
        while time.monotonic() < deadline:
            if self.request.wait_edge_events(1):
                events = self.request.read_edge_events()
                for ev in events:
                    if ev.line_offset == self.echo_pin and \
                            ev.event_type == gpiod.EdgeEvent.Type.RISING_EDGE:
                        pulse_start = ev.timestamp_ns * 1e-9
                        break
                else:
                    continue
                break
        else:
            return None

        # Wait for falling edge
        while time.monotonic() < deadline:
            if self.request.wait_edge_events(1):
                events = self.request.read_edge_events()
                for ev in events:
                    if ev.line_offset == self.echo_pin and \
                            ev.event_type == gpiod.EdgeEvent.Type.FALLING_EDGE:
                        pulse_end = ev.timestamp_ns * 1e-9
                        break
                else:
                    continue
                break
        else:
            return None

        pulse_duration = pulse_end - pulse_start
        if pulse_duration < 150e-6 or pulse_duration > 25e-3:
            return None

        distance = (pulse_duration * self.SPEED_OF_SOUND_CM_PER_S) / 2
        if not 2 <= distance <= 400:
            return None

        # Median + EMA filter
        self.raw_history.append(distance)
        if len(self.raw_history) < 3:
            return None

        median_dist = sorted(self.raw_history)[len(self.raw_history) // 2]
        if self.ema_value is None:
            self.ema_value = median_dist
        else:
            self.ema_value = (
                self.ema_alpha * median_dist +
                (1 - self.ema_alpha) * self.ema_value
            )

        return self.ema_value

    # -------------------------------------------------
    # Worker loop (process-safe)
    # -------------------------------------------------
    def measure_cont(self):
        self._setup_gpio()
        self.flag.set()

        try:
            while self.flag.is_set():
                start = time.perf_counter()

                dist = self.take_measurement()
                if dist is not None:
                    self.connector.send(dist)

                elapsed = time.perf_counter() - start
                sleep_time = self.period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            self._cleanup_gpio()

class LED:
    def __init__(self, command_connector: Connection, control_event: Event):
        self.command_connector = command_connector
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
            color_choice = int(round(self.rng.random()*2))
            color[color_choice] = self.rng.random()*255
            color_choice = int(round(self.rng.random() * 2))
            color[color_choice] = self.rng.random() * 255
            self.setSomeColor(*color, i)
        self.strip.show()
        time.sleep(0.5)

    def run_lights(self):
        while not self.stopped_flag.is_set():
            try:
                # Check if there are new commands
                if self.command_connector.poll():
                    command = recv_latest(self.command_connector)
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


def led_worker(command_connector: Connection, control_event: Event):
    led = LED(command_connector, control_event)
    control_event.clear()
    led.run_lights()


def distance_sensor_worker(distance_connector: Connection, control_event: Event,
                           gpio_chip="/dev/gpiochip0", trigger=23, echo=24):
    sensor = DistSensor(distance_connector, control_event,
                        gpio_chip=gpio_chip, GPIO_trigger=trigger,
                        GPIO_echo=echo)
    sensor.measure_cont()


def test_led():
    rcv_conn, send_conn = Pipe(duplex=False)
    control_event = Event()
    led_process = Process(target=led_worker, args=(rcv_conn, control_event))
    led_process.start()

    try:
        while True:
            print('all_good')
            send_conn.send(('all_good', True))  # (mode, breath)
            time.sleep(10)
            print('yellow_alert')
            send_conn.send(('yellow_alert', True))
            time.sleep(10)
            print('red_alert')
            send_conn.send(('red_alert', True))
            time.sleep(10)
            print('remote_controlled')
            send_conn.send(('remote_controlled', True))
            time.sleep(10)
            print('police')
            send_conn.send(('police', False))
            time.sleep(10)
            print('disco')
            send_conn.send(('disco', False))
            time.sleep(10)

    except KeyboardInterrupt:
        print("Exiting...")
        control_event.set()
        led_process.join()
        send_conn.close()
        rcv_conn.close()
        GPIO.cleanup()


def direct_led_check():
    rcv_conn, send_conn = Pipe(duplex=False)
    control_event = Event()
    led = LED(rcv_conn, control_event)
    led_process = Thread(target=led.run_lights)
    led_process.start()
    try:
        while True:
            print('all_good')
            send_conn.send(('all_good', True))  # (mode, breath)
            time.sleep(10)
            print('yellow_alert')
            send_conn.send(('yellow_alert', True))
            time.sleep(10)
            print('red_alert')
            send_conn.send(('red_alert', True))
            time.sleep(10)
            print('remote_controlled')
            send_conn.send(('remote_controlled', True))
            time.sleep(10)
            print('police')
            send_conn.send(('police', False))
            time.sleep(10)
            print('disco')
            send_conn.send(('disco', False))
            time.sleep(10)

    except KeyboardInterrupt:
        print("Exiting...")
        send_conn.close()
        rcv_conn.close()
        GPIO.cleanup()


def test_dist_sensor():
    rcv_conn, send_conn = Pipe(duplex=False)
    control_event = Event()
    dist_measure_process = Process(target=distance_sensor_worker, args=(send_conn, control_event), daemon=True)
    dist_measure_process.start()
    try:
        while True:
            if rcv_conn.poll():
                abstand = recv_latest(rcv_conn)
                print("Gemessene Entfernung = %.1f cm" % abstand)
            time.sleep(0.05)

        # Beim Abbruch durch STRG+C resetten
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        control_event.clear()  # Send stop signal!
        dist_measure_process.join(timeout=1)
        send_conn.close()
        rcv_conn.close()


if __name__ == '__main__':
    # direct_led_check()
    # test_led()
    test_dist_sensor()
