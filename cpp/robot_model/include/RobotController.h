#ifndef RobotController_h
#define RobotController_h
#include "Arduino.h"
#include "math.h"
#include "SpiderLeg.h"
#include "RobotModel.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

class RobotController {
  private:
  short current_gait_no;
  short current_pose_no;
  short cur_sample;
  short cur_step;
  bool init_flag;
  bool reset_flag;

  public:
  HardwareSerial* stream;
  RobotModel robot_model;
  SpiderLeg * leg_list[4];
  RobotController(SpiderLeg* leg_list[4], HardwareSerial* serial, Adafruit_PWMServoDriver* pwm, Adafruit_MPU6050* mpu);
  void run();
  void read_serial();
  void write_serial(String mssg);
  void get_current_gait_no();
  void get_current_pose_no();
};

#endif