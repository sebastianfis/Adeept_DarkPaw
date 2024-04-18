#include "RobotController.h"
#include "Arduino.h"
#include "RobotModel.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN   100   // 0 bis 4096 - try and error
#define SERVOMAX   560   // 0 bis 4096 - try and error
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates
#define DEBUG 1 //debug mode


//Create port list:
const short port_list[4][3] = {{6, 7, 8},
                               {0, 1, 2},
                               {9, 10, 11},
                               {3, 4, 5}};

// Create init_pwm list
const short init_pwm[4][3] = {{295, 285, 290},
                              {305, 325, 330},
                              {365, 340, 345},
                              {295, 300, 285}};

// Create actuator_direction list
const short actuator_direction[4][3] = {{1, -1, 1},
                                        {-1, 1, -1},
                                        {1, 1, -1},
                                        {-1, -1, 1}}; 

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
    this->bpm_setting = 60;
    this->velocity_setting = 100;
    this->change_v_flag = false;
    this->init_flag = false;
    this->reset_flag = false;
    this->pose_reached_flag = false;
    this->dance_flag = false;
    this->balance_flag = false;
    this->pwm = pwm;
    this->mpu = mpu;
}

void RobotController::init(){
    this->stream->begin(115200);
    this->pwm_update_period = short(round(1e6/SERVO_FREQ));
    this->time_now = 0;
    this->pwm->begin();
    this->pwm->setPWMFreq(SERVO_FREQ);
    if (DEBUG) {
         this->stream->println(String("PWM driver succesfully initialized!"));
    }
    this->mpu->begin();
    this->mpu->setGyroRange(MPU6050_RANGE_250_DEG);
    this->mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);
    if (DEBUG) {
         this->stream->println(String("MPU6050 sensor succesfully initialized!"));
    }
    // init pwm for actuators
    for (short leg_no = 0; leg_no < 4; ++leg_no){
        this->leg_list[leg_no]->actuator1.set_pwm_init(init_pwm[leg_no][0],actuator_direction[leg_no][0]);
        this->leg_list[leg_no]->actuator2.set_pwm_init(init_pwm[leg_no][1],actuator_direction[leg_no][1]);
        this->leg_list[leg_no]->actuator3.set_pwm_init(init_pwm[leg_no][2],actuator_direction[leg_no][2]);
        if (DEBUG){
            this->stream->println(String("leg actuators of ") + this->leg_list[leg_no]->get_name() + " initialized as: (" + init_pwm[leg_no][0] + ", " +
                                                                                                                            init_pwm[leg_no][1] + ", " +
                                                                                                                            init_pwm[leg_no][2] + ")");
                }       
    }
    if (DEBUG) {
         this->stream->println(String("leg actuators succesfully initialized!"));
    }
    this->current_pose_no = 0;
    this->robot_model.init();
    if (DEBUG) {
      Serial.println(String("Robot model succesfully initialized"));
    }   
}

void RobotController::execute_reset(){
    float coord_value[3];
    short pwm_value[3];
    if ((micros() - time_now)>=this->pwm_update_period) {
        time_now = micros();
        if (DEBUG){
            this->stream->println(String("reset step no ") + cur_step + ":");
        }
        for (short leg_no = 0; leg_no < 4; ++leg_no){
            for (short ii = 0; ii < 3; ++ii){
                coord_value[ii]=this->robot_model.get_reset_coord(this->cur_step,this->cur_sample,leg_no, ii);
                pwm_value[ii]=this->robot_model.get_reset_pwm(this->cur_step,this->cur_sample,leg_no, ii);
            }
            if (coord_value[0] > -900){
                robot_model.leg_list[leg_no]->update_cur_phi(coord_value[0],coord_value[1], coord_value[2]);
                if (DEBUG){
                    this->stream->println(String("coordinates of ") + this->leg_list[leg_no]->get_name() + " updated to: (" + coord_value[0] + ", " +
                                                                                                                            coord_value[1] + ", " +
                                                                                                                            coord_value[2] + ")");
                }
                for (short ii = 0; ii < 3; ++ii){
                    pwm->setPWM(port_list[leg_no][ii], 0, pwm_value[ii]);
                    
                }
                if (DEBUG){
                    this->stream->println(String("Set pwm of") + this->leg_list[leg_no]->get_name() + "actuators to: (" + pwm_value[0] + ", " +
                                                                                                                        pwm_value[1] + ", " +
                                                                                                                        pwm_value[2] + ")");
                }
            }
        }
        this->cur_sample++;
        if (coord_value[0] < -900 || this->cur_sample == 8){
            this->cur_step++;
            this->cur_sample = 0;
            if (this->cur_step == 5) {
                this->reset_flag = false;
                this->cur_step = 0;
            }
        }
    }
}

void RobotController::dance(){
    //implement dance!
}

void RobotController::execute_gait(){
    //implement walking!
}

void RobotController::execute_pose(){
    //implement gait set!
}

void RobotController::balance(){
    //implement stabilization control
}

void RobotController::initiate_reset_step(){
    if (this->current_gait_no >= 0){
            this->robot_model.calc_reset_step();
            this->reset_flag = true;
        }
}

void RobotController::run(){
    this->read_serial();
    if (this->change_v_flag){
        this->robot_model.set_velocity(this->velocity_setting);
        this->change_v_flag=false;
    }
    while (this->reset_flag) {
        this->execute_reset();
    }
    if (this->balance_flag){
        this->balance();
    }
    if (this->dance_flag){
        this->dance();
    }
    if (this->current_gait_no >= 0){
        this->execute_gait();
    }
    if (this->current_pose_no >= 0 && !this->pose_reached_flag){
        this->execute_pose();
    }
}

void RobotController::read_serial(){
    String mssg; //check if this can't be done using chars...
    short new_gait = -1;
    short new_pose = -1;
    mssg = this->stream->readStringUntil(';');
     this->stream->flush();
    if (mssg.startsWith(String("v"))){
        mssg.remove(0,1);
        this->change_v_flag = true;
        this->velocity_setting = short(mssg.toInt());
    }
    if (mssg.startsWith(String("g"))){
        this->balance_flag = false;
        this->dance_flag = false;
        if (mssg.charAt(1) == 'm'){
            if (mssg.charAt(2) == 'f'){
                new_gait = 0;
            }
            else if(mssg.charAt(2) == 'b'){
                new_gait = 1;
            }
            else if(mssg.charAt(2) == 'r'){
                new_gait = 2;
            }
            else if(mssg.charAt(2) == 'l'){
                new_gait = 3;
            }
        }
        else if (mssg.charAt(1) == 't'){
            if (mssg.charAt(2) == 'r'){
                new_gait = 4;
            }
            if (mssg.charAt(2) == 'l'){
                new_gait = 5;
            }
        }
    }

    if (mssg.startsWith(String("p"))){
        this->balance_flag = false;
        this->dance_flag = false;
        if (mssg.charAt(1) == 'n'){
            new_pose = 0;
        }
        else if (mssg.charAt(1) == 'l'){
            if (mssg.charAt(2) == 'u'){
                new_pose = 1;
            }
            if (mssg.charAt(2) == 'd'){
                new_pose = 2;
            }
            if (mssg.charAt(2) == 'r'){
                new_pose = 3;
            }
            if (mssg.charAt(2) == 'l'){
                new_pose = 4;
            }
            if (mssg.charAt(2) == 'o'){
                new_pose = 6;
            }
        }
        else if (mssg.charAt(1) == 'h'){
            new_pose = 5;
        }
    }
    if (mssg.startsWith(String("d"))){
        this->dance_flag = true;
        this->balance_flag = false;
        mssg.remove(0,1);
        short bpm_value = short(mssg.toInt());
        if (bpm_value < 40){
            bpm_value = 40;
        }
        else if (bpm_value > 160){
            bpm_value = 160;
        }
        this->bpm_setting = bpm_value;
    }

    if (mssg.startsWith(String("s"))){
        this->initiate_reset_step();
        this->current_pose_no = -1;
        this->current_gait_no = -1;
        this->balance_flag = false;
        this->dance_flag = false;

    }

    if (mssg.startsWith(String("b"))){
        this->initiate_reset_step();
        this->current_pose_no = -1;
        this->current_gait_no = -1;
        this->balance_flag = true;
        this->dance_flag = false;

    }


    // if current order changes:
    if (new_gait != this->current_gait_no && new_gait >=0){
        this->initiate_reset_step();
        this->init_flag = true;
        this->current_gait_no = new_gait;
        this->current_pose_no = -1;
    }
    else if(new_pose != this->current_pose_no && new_pose >=0){
        this->initiate_reset_step();
        this->init_flag = false;
        this->pose_reached_flag = false;
        this->current_gait_no = -1;
        this->current_pose_no = new_pose;
    }
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