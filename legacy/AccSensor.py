from mpu6050 import mpu6050
import time
import numpy as np
# NOTE: ACC-Sensor input will go directly into the ESP, so python code is not needed!

class AccSensor:
    def __init__(self):
        self.sensor = mpu6050(0x68)

    def read_all_sensor_data(self, average_num=3):
        # Read the accelerometer values
        accelerometer_data = self.read_accel_data(average_num)
        # Read the gyroscope values
        gyroscope_data = self.read_gyro_data(average_num)
        # Read temp
        temperature = 0
        for i in range(0, average_num):
            temperature += self.sensor.get_temp()
        temperature /= average_num
        return accelerometer_data, gyroscope_data, temperature

    def read_accel_data(self, average_num=3):
        acc_data = {'x': 0, 'y': 0, 'z': 0}
        for i in range(0, average_num):
            accelerometer_data = self.read_accel_data()
            acc_data['x'] += accelerometer_data['x']
            acc_data['y'] += accelerometer_data['y']
            acc_data['z'] += accelerometer_data['z']
        for direction in acc_data.keys():
            acc_data[direction] /= average_num
        # due to sensor mounting y and z must be inverted!
        acc_data['y'] *= -1
        acc_data['z'] *= -1
        return acc_data

    def read_gyro_data(self, average_num=3):
        gyro_data = {'x': 0, 'y': 0, 'z': 0}
        for i in range(0, average_num):
            gyroscope_data = self.sensor.get_gyro_data()
            gyro_data['x'] += gyroscope_data['x']
            gyro_data['y'] += gyroscope_data['y']
            gyro_data['z'] += gyroscope_data['z']
        for direction in gyro_data.keys():
            gyro_data[direction] /= average_num
        # due to sensor mounting y and z must be inverted!
        gyro_data['y'] *= -1
        gyro_data['z'] *= -1
        return gyro_data

    def read_body_angles(self):
        accel_data = self.read_accel_data()
        theta_x = np.rad2deg(np.arctan2(accel_data['x']/accel_data['z']))
        theta_y = np.rad2deg(np.arctan2(accel_data['y']/accel_data['z']))
        return theta_x, theta_y

    def funtion_test(self, average_num=3):
        try:
            while True:
                print('Taking measurement with averaging over % samples' % average_num)
                accelerometer_data, gyroscope_data, temperature = self.read_all_sensor_data(average_num)

                print('Measured acceleration: X=%.3f m/s², Y=%.3f m/s², Z=%.3f m/s²' % (accelerometer_data['x'],
                                                                                        accelerometer_data['x'],
                                                                                        accelerometer_data['x']))
                print('Measured angle velocity: X=%.3f deg/s, Y=%.3f deg/s, Z=%.3f deg/s' % (gyroscope_data['x'],
                                                                                             gyroscope_data['x'],
                                                                                             gyroscope_data['x']))
                print("Measured TEmperatre: %.1f deg C" % temperature)
                time.sleep(1)

            # Beim Abbruch durch STRG+C resetten
        except KeyboardInterrupt:
            print("Messung vom User gestoppt")
