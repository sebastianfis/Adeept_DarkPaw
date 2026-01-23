import time
import numpy as np
import gpiod
import RPi.GPIO as GPIO
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver
from threading import Event, Thread
from multiprocessing.connection import Connection
from multiprocessing import Process, Pipe
import psutil
import os
from collections import deque

"""
parent_conn, child_conn = Pipe(duplex=False)

    parent_conn → sender

    child_conn → receiver
"""


def send_latest(conn, data):
    # Non-blocking-ish: drop old unread data
    while conn.poll():
        try:
            conn.recv()
        except EOFError:
            break
    conn.send(data)


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
        gpio_chip: str = "gpiochip0",
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

        self.raw_history = deque(maxlen=5)  # median window
        self.ema_value = None
        self.ema_alpha = 0.3  # 0.2–0.4 is good

        self.chip = None
        self.trigger = None
        self.echo = None

    # -------------------------------------------------
    # GPIO setup / cleanup (must be inside the process)
    # -------------------------------------------------
    def _setup_gpio(self):
        self.chip = gpiod.Chip(self.chip_name)

        self.trigger = self.chip.get_line(self.trigger_pin)
        self.echo = self.chip.get_line(self.echo_pin)

        self.trigger.request(
            consumer="dist_sensor",
            type=gpiod.LINE_REQ_DIR_OUT
        )
        self.trigger.set_value(0)

        self.echo.request(
            consumer="dist_sensor",
            type=gpiod.LINE_REQ_EV_BOTH_EDGES
        )

    def _cleanup_gpio(self):
        if self.trigger:
            self.trigger.release()
        if self.echo:
            self.echo.release()
        if self.chip:
            self.chip.close()

    # -------------------------------------------------
    # Single blocking measurement (edge-based)
    # -------------------------------------------------
    def take_measurement(self):
        # Flush stale events
        while self.echo.event_wait(0):
            self.echo.event_read()

        # Trigger pulse (15 µs)
        self.trigger.set_value(1)
        time.sleep(15e-6)
        self.trigger.set_value(0)

        deadline = time.monotonic() + self.TIMEOUT_S

        # Wait for rising edge
        while time.monotonic() < deadline:
            if self.echo.event_wait(1):
                event1 = self.echo.event_read()
                if event1.type == gpiod.LineEvent.RISING_EDGE:
                    break
        else:
            return None

        # Wait for falling edge
        while time.monotonic() < deadline:
            if self.echo.event_wait(1):
                event2 = self.echo.event_read()
                if event2.type == gpiod.LineEvent.FALLING_EDGE:
                    break
        else:
            return None

        pulse_start = event1.sec + event1.nsec * 1e-9
        pulse_end = event2.sec + event2.nsec * 1e-9
        pulse_duration = pulse_end - pulse_start

        if pulse_duration < 150e-6 or pulse_duration > 25e-3:
            return None

        distance = (pulse_duration * self.SPEED_OF_SOUND_CM_PER_S) / 2

        if not 2 <= distance <= 400:
            return None

        # Median + EMA
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
                    send_latest(self.connector, dist)

                # enforce measurement period
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
                    command = self.command_connector.recv()
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
                           gpio_chip="gpiochip0", trigger=23, echo=24):
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
            send_latest(send_conn, ('all_good', True))  # (mode, breath)
            time.sleep(10)
            print('yellow_alert')
            send_latest(send_conn, ('yellow_alert', True))
            time.sleep(10)
            print('red_alert')
            send_latest(send_conn, ('red_alert', True))
            time.sleep(10)
            print('remote_controlled')
            send_latest(send_conn, ('remote_controlled', True))
            time.sleep(10)
            print('police')
            send_latest(send_conn, ('police', False))
            time.sleep(10)
            print('disco')
            send_latest(send_conn, ('disco', False))
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
            send_latest(send_conn, ('all_good', True))  # (mode, breath)
            time.sleep(10)
            print('yellow_alert')
            send_latest(send_conn, ('yellow_alert', True))
            time.sleep(10)
            print('red_alert')
            send_latest(send_conn, ('red_alert', True))
            time.sleep(10)
            print('remote_controlled')
            send_latest(send_conn, ('remote_controlled', True))
            time.sleep(10)
            print('police')
            send_latest(send_conn, ('police', False))
            time.sleep(10)
            print('disco')
            send_latest(send_conn, ('disco', False))
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
                abstand = rcv_conn.recv()
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
