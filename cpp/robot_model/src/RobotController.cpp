#include "RobotController.h"
#include "Arduino.h"
#include "RobotModel.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN   68   // 0 bis 4096 - try and error
#define SERVOMAX   510  // 0 bis 4096 - try and error
#define SERVO_FREQ 50   // Analog servos run at ~50 Hz updates 

RobotController::RobotController(SpiderLeg *leg_list[4], HardwareSerial* serial, Adafruit_PWMServoDriver* pwm, Adafruit_MPU6050* mpu):
robot_model(leg_list) {
    this->stream = serial;
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
    this->current_gait_no = -1;
    this->current_pose_no = -1;
    this->cur_sample = 0;
    this->cur_step = 0;
    this->init_flag = false;
    this->reset_flag = false;
}

void RobotController::run(){
    while (true) {

    }
}

void RobotController::read_serial(){
    String mssg;
    short new_gait = -1;
    short new_pose = -1;
    mssg = this->stream->readStringUntil(';');
}

void RobotController::write_serial(String mssg){
      this->stream->println(mssg);
    }

void RobotController::get_current_gait_no() { 
    char* name;
    if (0 < this->current_gait_no < 6) {
        this->robot_model.gait_list[current_gait_no]->get_name(name);
        this->write_serial(String("current gait: ") + name);
    }
    else {
        this->write_serial(String("no gait currently running"));
    }
}

void RobotController::get_current_pose_no(){
    if (0 < this->current_pose_no < 7) {
        this->write_serial(String("current pose: ") + this->robot_model.pose_list[current_pose_no]->get_name());
    }
    else {
        this->write_serial(String("no pose currently running"));
    }
}
