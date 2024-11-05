#include "Arduino.h"
#include "SpiderLeg.h"
#include "RobotController.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
// #include "FunctionTests.h"

SpiderLeg RF(1, 1, "RF");
SpiderLeg LF(1, -1, "LF");
SpiderLeg RB(-1, 1, "RB");
SpiderLeg LB(-1, -1, "LB");
SpiderLeg *leg_list[] = {&RF, &LF, &RB, &LB};

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
RobotController robot_control(leg_list, &Serial, &pwm, &mpu);


void setup() {
  robot_control.init();
  // put your setup code here, to run once:
}


void loop() {
  robot_control.run();
  // run tests(&robot_control.robot_model);
  // put your main code here, to run repeatedly:
}
