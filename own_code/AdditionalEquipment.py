from mpu6050 import mpu6050
import time
import RPi.GPIO as GPIO
from rpi_ws281x import Adafruit_NeoPixel, Color
from threading import Event

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# LED strip configuration:
LED_COUNT      = 3      # Number of LED pixels.
LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

class AccSensor:
    def __init__(self):
        self.sensor = mpu6050(0x68)

    def function_test(self):
        x = 0
        y = 0
        z = 0
        for i in range(0, 10):
            accelerometer_data = self.read_accel_data()
            x = x + accelerometer_data['x']
            y = y + accelerometer_data['y']
            z = z + accelerometer_data['z']
        print('X=%.3f, Y=%.3f, Z=%.3f' % (x / 9.81, y / 9.81, z / 9.81))
        time.sleep(0.3)

    def read_all_sensor_data(self):
        # Read the accelerometer values
        accelerometer_data = self.sensor.get_accel_data()

        # Read the gyroscope values
        gyroscope_data = self.sensor.get_gyro_data()

        # Read temp
        temperature = self.sensor.get_temp()

        return accelerometer_data, gyroscope_data, temperature

    def read_accel_data(self):
        return self.sensor.get_accel_data()

    def read_gyro_data(self):
        return self.sensor.get_gyro_data()


class DistSensor:
    def __init__(self, GPIO_trigger: int = 18, GPIO_echo: int = 24, cont_measurement_timer: int = 100):
        self.trigger = GPIO_trigger
        self.echo = GPIO_echo
        self.last_measurement = 0
        self.cont_measurement_timer = cont_measurement_timer
        self.cont_measurement_flag = Event()
        self.cont_measurement_flag.clear()
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
                self.last_measurement = self.take_measurement()
                last_exec_time = now_time


class LED:
    def __init__(self, led_pin: int = 5):
        self.led_count = 7      # Number of LED pixels.
        self.led_pin = led_pin      # GPIO pin connected to the pixels (18 uses PWM!).
        self.led_freq_hz = 800000  # LED signal frequency in hertz (usually 800khz)
        self.led_dma = 10      # DMA channel to use for generating signal (try 10)
        self.led_brightness = 255     # Set to 0 for darkest and 255 for brightest
        self.led_invert = False   # True to invert the signal (when using NPN transistor level shift)
        self.led_channel = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.led_count, self.led_pin, self.led_freq_hz, self.led_dma, self.led_invert, self.led_brightness, self.led_channel)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

        self.colorBreathR = 0
        self.colorBreathG = 0
        self.colorBreathB = 0
        self.breathSteps = 10

        self.lightMode = 'none'  # 'none' 'police' 'breath'

        GPIO.setwarnings(False)
        GPIO.setup(self.led_pin, GPIO.OUT)


    # Define functions which animate LEDs in various ways.
    def setColor(self, R, G, B):
        """Wipe color across display a pixel at a time."""
        color = Color(int(R), int(G), int(B))
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()

    def light_off(self):
        self.setColor(0, 0, 0)

    def all_good(self):
        self.setColor(0, 0, 255)

    def yellow_alert(self):
        self.setColor(255, 191, 0)

    def red_alert(self):
        self.setColor(255, 0, 0)

    def setSomeColor(self, R, G, B, ID):
        color = Color(int(R), int(G), int(B))
        # print(int(R),'  ',int(G),'  ',int(B))
        for i in ID:
            self.strip.setPixelColor(i, color)
            self.strip.show()

