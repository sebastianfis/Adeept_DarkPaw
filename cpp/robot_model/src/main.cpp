#include "Arduino.h"
#include "SpiderLeg.h"
#include "RobotController.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

/**
 * @brief Pin to enable the robot electronics
 */
const int EnablePin = 25;    // the number of the Enable Pin

/**
 * @brief Define the four legs of the robot
 *
 * RF = Right Front
 * LF = Left Front
 * RB = Right Back
 * LB = Left Back
 */
SpiderLeg RF(1, 1, "RF");
SpiderLeg LF(1, -1, "LF");
SpiderLeg RB(-1, 1, "RB");
SpiderLeg LB(-1, -1, "LB");
/**
 * @brief Array of pointers to all SpiderLeg objects
 */
SpiderLeg *leg_list[] = {&RF, &LF, &RB, &LB};

/**
 * @brief MPU6050 IMU sensor
 */
Adafruit_MPU6050 mpu;
/**
 * @brief PWM servo driver
 */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/**
 * @brief Main robot controller object
 *
 * Initialized with leg list, serial port, PWM driver, and IMU sensor.
 */
RobotController robot_control(leg_list, &Serial, &pwm, &mpu);


/**
 * @brief ESP32 setup function
 *
 * Initializes the robot controller and sets the enable pin HIGH to power actuators.
 */
void setup() {
  robot_control.init();
  pinMode(EnablePin, OUTPUT);
  digitalWrite(EnablePin, HIGH);
}


/**
 * @brief ESP32 main loop
 *
 * Continuously calls the robot controller's run method.
 * Uncomment run_tests() to perform diagnostic tests on the robot model.
 */
void loop() {
  robot_control.run();
  // run_tests(&robot_control.robot_model); // Uncomment for testing routines
}
