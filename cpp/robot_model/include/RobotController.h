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
  short current_gait_no, current_pose_no, cur_sample, cur_step;
  short velocity_setting, bpm_setting;
  float cur_theta_x, cur_theta_y, acc_x_error, acc_y_error, gyro_x_error, gyro_y_error;
  bool change_v_flag, init_flag, reset_flag, pose_reached_flag, dance_flag, balance_flag;
  unsigned long time_now, pwm_update_period, sensor_timer;

  public:
  HardwareSerial* stream;
  RobotModel robot_model;
  SpiderLeg* leg_list[4];
  Adafruit_PWMServoDriver* pwm; 
  Adafruit_MPU6050* mpu;
  RobotController(SpiderLeg* leg_list[4], HardwareSerial* serial, Adafruit_PWMServoDriver* pwm, Adafruit_MPU6050* mpu);
  void init();
  void run();
  void read_serial();
  void write_serial(String mssg);
  void get_current_gait_no();
  void get_current_pose_no();
  void execute_reset();
  void dance();
  void execute_gait();
  void execute_pose();
  void balance();
  void evaluate_mpu_errors();
  void initiate_reset_step();
};

#endif