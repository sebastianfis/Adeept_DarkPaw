from mpu6050 import mpu6050
import time
import RPi.GPIO as GPIO

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

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
    def __init__(self,GPIO_trigger: int=18, GPIO_echo: int=24):
        self.trigger = GPIO_trigger
        self.echo = GPIO_echo
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
