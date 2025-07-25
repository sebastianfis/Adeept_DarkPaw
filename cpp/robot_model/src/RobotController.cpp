#include "RobotController.h"
#include "Arduino.h"
#include "RobotModel.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <sstream>

#define SERVOMIN   100   // 0 bis 4096 - try and error
#define SERVOMAX   560   // 0 bis 4096 - try and error
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates
#define EMEA 0.1 // Measurement Uncertainty for Kalman Filter
#define EEST 0.01 // Estimation Uncertainty for Kalman Filter
#define PROCNOISE 0.03 // Process Noise for Kalman Filter
#define P 1 // proportional parameter for controller
#define I 0.00001 // integral parameter for controller
// #include "FunctionTests.h"
#define DEBUG 1 //debug mode


const byte numChars = 10;
char receivedChars[numChars];   // an array to store the received data

Preferences preferences;

// Create port list:
const short port_list[4][3] = {{9, 8, 7},   //{{6, 7, 8},
                              {15, 14, 13}, // {0, 1, 2},
                              {6, 5, 4},    // {9, 10, 11},
                              {12, 11, 10}}; // {3, 4, 5}};

// Create init_pwm list
const short init_pwm[4][3] = {{275, 280, 295},
                              {290, 290, 270},
                              {340, 315, 325},
                              {275, 305, 310}};

// Create actuator_direction list
const short actuator_direction[4][3] = {{1, -1, 1},
                                        {-1, 1, -1},
                                        {1, 1, -1},
                                        {-1, -1, 1}}; 

RobotController::RobotController(SpiderLeg *leg_list[4], 
                                 HardwareSerial* serial, 
                                 Adafruit_PWMServoDriver* pwm, 
                                 Adafruit_MPU6050* mpu):
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
    this->bpm_setting = 80;
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
    this->pwm_update_period = round(1e6/SERVO_FREQ);
    this->time_now = 0;
    this->sensor_timer = 0;
    this->last_theta_x = 0;
    this->last_theta_y = 0;
    this->integral_x = 0;
    this->integral_y = 0;
    this->pwm->begin();
    this->pwm->setOscillatorFrequency(24700000);
    this->pwm->setPWMFreq(SERVO_FREQ);

    if (DEBUG) {
         this->stream->println(String("PWM driver succesfully initialized!"));
    }

    preferences.begin("act_data", false);

    for (short leg_no = 0; leg_no < 4; ++leg_no){
        for (short act_no = 0; act_no <3; ++act_no) {
            if (!preferences.isKey((String("init_pwm_") + leg_no + "" + act_no).c_str())) {
                preferences.putShort((String("init_pwm_") + leg_no + "" + act_no).c_str(), init_pwm[leg_no][act_no]);
            }
            if (!preferences.isKey((String("act_dir_") + leg_no + "" + act_no).c_str())) {
                preferences.putShort((String("act_dir_") + leg_no + "" + act_no).c_str(), actuator_direction[leg_no][act_no]);
            }
        }
    }

    this->set_init_pwm();

    delay(100);
    this->mpu->begin();
    this->mpu->setGyroRange(MPU6050_RANGE_250_DEG);
    this->mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);
    this->evaluate_mpu_errors();
    if (DEBUG) {
         this->stream->println(String("MPU6050 sensor succesfully initialized!"));
    }
}

void RobotController::set_init_pwm(){
    // init pwm for actuators
    for (short leg_no = 0; leg_no < 4; ++leg_no){
        this->leg_list[leg_no]->actuator1.set_pwm_init(preferences.getShort((String("init_pwm_") + leg_no + "0").c_str()), 
                                                       preferences.getShort((String("act_dir_") + leg_no + "0").c_str()));
            // init_pwm[leg_no][0],actuator_direction[leg_no][0]);
        this->leg_list[leg_no]->actuator2.set_pwm_init(preferences.getShort((String("init_pwm_") + leg_no + "1").c_str()),
                                                       preferences.getShort((String("act_dir_") + leg_no + "1").c_str()));
            // init_pwm[leg_no][1],actuator_direction[leg_no][1]);
        this->leg_list[leg_no]->actuator3.set_pwm_init(preferences.getShort((String("init_pwm_") + leg_no + "2").c_str()),
                                                       preferences.getShort((String("act_dir_") + leg_no + "2").c_str()));
            // init_pwm[leg_no][2],actuator_direction[leg_no][2]);
        if (DEBUG){
            this->stream->println(String("leg actuators of ") + this->leg_list[leg_no]->get_name() + " initialized as: (" + 
                                  preferences.getShort((String("init_pwm_") + leg_no + "0").c_str()) + ", " +
                                  preferences.getShort((String("init_pwm_") + leg_no + "1").c_str()) + ", " +
                                  preferences.getShort((String("init_pwm_") + leg_no + "2").c_str()) + ")");
                }       
    }
    if (DEBUG) {
         this->stream->println(String("leg actuators succesfully initialized!"));
    }
    this->current_pose_no = 0;
    this->robot_model.init();
    if (DEBUG) {
      this->stream->println(String("Robot model succesfully initialized"));
    }

    while(!this->pose_reached_flag){
        this->execute_pose();
    }
}

void RobotController::change_init_pwm(short port, short init_pwm_value){
    for (short leg_no = 0; leg_no < 4; ++leg_no){
        for (short act_no = 0; act_no <3; ++act_no) {
            if (port_list[leg_no][act_no] == port){
                preferences.putShort((String("init_pwm_") + leg_no + "" + act_no).c_str(), init_pwm_value);
                return;
            }
        }
    }
}

void RobotController::change_act_dir(short port, short act_dir_value){
    for (short leg_no = 0; leg_no < 4; ++leg_no){
        for (short act_no = 0; act_no <3; ++act_no) {
            if (port_list[leg_no][act_no] == port){
                preferences.putShort((String("act_dir_") + leg_no + "" + act_no).c_str(), act_dir_value);
                return;
            }
        }
    }
}

void RobotController::evaluate_mpu_errors(){
    short ii =0;
    sensors_event_t a, g, temp;
    float acc_x, acc_y, ycc_z, gyro_x, gyro_y, gyro_z;
    float AccErrorX = 0;
    float AccErrorY = 0;
    float GyroErrorX = 0;
    float GyroErrorY = 0;
    while (ii < 200){
        this->mpu->getEvent(&a, &g, &temp);
        // Sum all readings
        AccErrorX = AccErrorX + ((atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
        GyroErrorX = GyroErrorX + g.gyro.x;
        GyroErrorY = GyroErrorY + g.gyro.y;
        ii++;
    }
    this->acc_x_error = AccErrorX/200;
    this->acc_y_error = AccErrorY/200;
    this->gyro_x_error = GyroErrorX/200;
    this->gyro_y_error = GyroErrorX/200;
}

void RobotController::execute_reset(){
    float coord_value[3];
    short pwm_value[3];
    if ((micros() - this->time_now)>=this->pwm_update_period) {
        this->time_now = micros();
        if (DEBUG){
            this->stream->println(String("Reset step no ") + this->cur_step + ", Sample no " + this->cur_sample);
        }
        for (short leg_no = 0; leg_no < 4; ++leg_no){
            for (short ii = 0; ii < 3; ++ii){
                coord_value[ii]=this->robot_model.get_reset_coord(this->cur_step, this->cur_sample, leg_no, ii);
                pwm_value[ii]=this->robot_model.get_reset_pwm(this->cur_step, this->cur_sample, leg_no, ii);
            }
            if (coord_value[0] > -900){
                robot_model.leg_list[leg_no]->update_cur_phi(coord_value[0],coord_value[1], coord_value[2]);
                // if (DEBUG){
                //     this->stream->println(String("coordinates of ") + this->leg_list[leg_no]->get_name() + " updated to: (" + coord_value[0] + ", " +
                //                                                                                                             coord_value[1] + ", " +
                //                                                                                                             coord_value[2] + ")");
                // }
                for (short ii = 0; ii < 3; ++ii){
                    pwm->setPWM(port_list[leg_no][ii], 0, pwm_value[ii]);
                }
                // if (DEBUG){
                //     this->stream->println(String("Set pwm of ") + this->leg_list[leg_no]->get_name() + " actuators to: (" + pwm_value[0] + ", " +
                //                                                                                                         pwm_value[1] + ", " +
                //                                                                                                         pwm_value[2] + ")");
                // }
            }
        }
        this->cur_sample++;
        if (coord_value[0] < -900 || this->cur_sample == 8){
            this->cur_step++;
            this->cur_sample = 0;
            if (this->cur_step == 5) {
                this->reset_flag = false;
                this->cur_step = 0;
                if (this->current_pose_no>=0){
                    if (DEBUG){
                        this->stream->println(String("Setting pose to ") + this->robot_model.pose_list[this->current_pose_no]->get_name() + ".");
                    }
                    this->robot_model.pose_list[this->current_pose_no]->calc_pose_lists(short(2));
                }
            }
        }
    }
}

void RobotController::dance(){
    unsigned long dance_update = round(1e6/short(this->bpm_setting)*60);
    short new_pose = -1;
    if ((micros() - this->time_now)>= dance_update) {
        this->time_now = micros();
        if (this->current_gait_no >= 0){
            this->initiate_reset_step();
            this->current_gait_no = -1;
        }
        new_pose = random(0, 7);
        // make sure new pose is different from current one!
        while (new_pose == this->current_pose_no){
            new_pose = random(0, 7);
        }
        this->current_pose_no = new_pose;
        this->cur_sample = 0;
        this->cur_step = 0;
        this->init_flag = false;
        this->pose_reached_flag = false;
    }

}

void RobotController::execute_gait(){
    float coord_value[3];
    short pwm_value[3];

    if ((micros() - this->time_now)>=this->pwm_update_period) {
        this->time_now = micros();
        if (DEBUG){
            if (this->init_flag){
                this->stream->println(String("Init step no ") + this->cur_step + ", Sample no " + this->cur_sample);
            }
            else{
                this->stream->println(String("Step no ") + this->cur_step + ", Sample no " + this->cur_sample);
            }
        }
        for (short leg_no = 0; leg_no < 4; ++leg_no){
            for (short ii = 0; ii < 3; ++ii){
                coord_value[ii]=this->robot_model.gait_list[this->current_gait_no]->get_coordinate_from_list(this->cur_step,this->cur_sample,leg_no, ii,this->init_flag);
                pwm_value[ii]=this->robot_model.gait_list[this->current_gait_no]->get_pwm_from_list(this->cur_step,this->cur_sample,leg_no, ii,this->init_flag);
            }
            if (coord_value[0] > -900){
                robot_model.leg_list[leg_no]->update_cur_phi(coord_value[0],coord_value[1], coord_value[2]);
                // if (DEBUG){
                //     this->stream->println(String("coordinates of ") + this->leg_list[leg_no]->get_name() + " updated to: (" + coord_value[0] + ", " +
                //                                                                                                             coord_value[1] + ", " +
                //                                                                                                             coord_value[2] + ")");
                // }
                for (short ii = 0; ii < 3; ++ii){
                    pwm->setPWM(port_list[leg_no][ii], 0, pwm_value[ii]);
                    
                }
                // if (DEBUG){
                //     this->stream->println(String("Set pwm of ") + this->leg_list[leg_no]->get_name() + " actuators to: (" + pwm_value[0] + ", " +
                //                                                                                                         pwm_value[1] + ", " +
                //                                                                                                         pwm_value[2] + ")");
                // }
            }
        }
        this->cur_sample++;
        if (coord_value[0] < -900 || this->cur_sample == 25){
            this->cur_step++;
            this->cur_sample = 0;
            if (this->init_flag && this->cur_step == 3) {
                this->init_flag = false;
                this->cur_step = 0;
            }
            else if(!this->init_flag && this->cur_step == 4){
                this->cur_step = 0;
            }
                
        }
    }
}

void RobotController::execute_pose(){
    float coord_value[3];
    short pwm_value[3];
    
    if (!this->pose_reached_flag){
        if ((micros() - this->time_now)>=this->pwm_update_period) {
            this->time_now = micros();
            if (DEBUG){
                this->stream->println(String("Sample no ") + this->cur_sample);
            }
            for (short leg_no = 0; leg_no < 4; ++leg_no){
                for (short ii = 0; ii < 3; ++ii){
                    coord_value[ii]=this->robot_model.pose_list[this->current_pose_no]->get_coordinate_from_list(this->cur_sample,leg_no, ii);
                    pwm_value[ii]=this->robot_model.pose_list[this->current_pose_no]->get_pwm_from_list(this->cur_sample,leg_no, ii);
                }
                if (coord_value[0] > -900){
                    robot_model.leg_list[leg_no]->update_cur_phi(coord_value[0],coord_value[1], coord_value[2]);
                    for (short ii = 0; ii < 3; ++ii){
                        pwm->setPWM(port_list[leg_no][ii], 0, pwm_value[ii]);
                        
                    }
                }
            }
            this->cur_sample++;
            if (coord_value[0] < -900 || this->cur_sample == 8){
                this->cur_sample = 0;
                this->pose_reached_flag = true;
                if (DEBUG && this->current_pose_no >= 0){
                    this->stream->println(String("Pose ") + this->robot_model.pose_list[this->current_pose_no]->get_name() + " reached!");
                }
            }
        }
    }
}

void RobotController::balance(){
    sensors_event_t a, g, temp;
    float movement_goal[4][3];
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
    float coordinates[3];
    float angles[3];
    short PWM_values[3];
    float kalman_gain = 0.1;//EEST / (EEST + EMEA);
    float theta_corx;
    float theta_cory;
    this->integral_x = 0;
    this->integral_y = 0;
    float ax_av =0;
    float ay_av =0;
    float az_av =0;

 
    unsigned long time_elapsed = micros() - this->sensor_timer; 
    if ( time_elapsed >= 2*this->pwm_update_period) {
        this->mpu->getEvent(&a, &g, &temp);
        accAngleX = ((atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI)) - this->acc_x_error;
        accAngleY = ((atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI)) - this->acc_y_error;
        this->cur_theta_y = this->last_theta_y + kalman_gain * (accAngleX - this->last_theta_y); //0.96 *gyroAngleX + 0.04 * accAngleX;
        this->cur_theta_x = this->last_theta_x + kalman_gain * (accAngleY - this->last_theta_x);// 0.96 *gyroAngleY + 0.04 * accAngleY;
        // Use PI finite differences control scheme
        this->integral_y = this->integral_y + 2*this->pwm_update_period*(this->last_theta_y + this->cur_theta_y)/2;
        this->integral_x = this->integral_x + 2*this->pwm_update_period*(this->last_theta_x + this->cur_theta_x)/2;
        theta_cory = P * this->cur_theta_y + I * this->integral_y;
        theta_corx = P * this->cur_theta_x + I * this->integral_x;
        // limit output
        if (theta_cory < -5.5) {
            theta_cory = -5.5;
        }
        else if (theta_cory > 5.5) {
            theta_cory = 5.5;
        }
        if (theta_corx < -10) {
            theta_corx = -10;
        }
        else if (theta_corx > 10) {
            theta_corx = 10;
        }
        //update last values
        this->last_theta_y = this->cur_theta_y;
        this->last_theta_x = this->cur_theta_x;

        if (DEBUG){
            this->stream->println(String("Measured angles: Theta_x = ") + accAngleY + " deg, Theta_y = " + accAngleX + " deg");
        }
        this->sensor_timer = micros();
        this->robot_model.calc_leg_pos_from_body_angles(movement_goal, theta_corx, -theta_cory, 0);
        for (short leg_no = 0; leg_no < 4; ++leg_no){
            for (short ii = 0; ii < 3; ++ii){
                    coordinates[ii]=movement_goal[leg_no][ii];
                }
            this->leg_list[leg_no]->calc_trajectory(&coordinates, &angles, 1);
            this->leg_list[leg_no]->calc_PWM(&angles, &PWM_values, 1);
            robot_model.leg_list[leg_no]->update_cur_phi(coordinates[0],coordinates[1], coordinates[2]);
            for (short ii = 0; ii < 3; ++ii){
                pwm->setPWM(port_list[leg_no][ii], 0, PWM_values[ii]);        
                }
            }
    }
}

void RobotController::initiate_reset_step(){
    if (this->current_gait_no >= 0){
            this->cur_step = 0;
            this->cur_sample = 0;
            this->robot_model.calc_reset_step();
            this->reset_flag = true;
        }
}

void RobotController::run(){
    this->read_serial();
    if (this->change_v_flag){
        this->robot_model.set_velocity(this->velocity_setting);
        if (DEBUG){
            this->stream->println(String("Set velocity to ") + this->velocity_setting + " %");
            }
        this->change_v_flag=false;
    }
    if (this->balance_flag){
        this->balance();
    }
    if (this->dance_flag){
        this->dance();
    }
    while (this->reset_flag) {
        this->execute_reset();
    }

    if (this->current_gait_no >= 0){
        this->execute_gait();
    }
    if (this->current_pose_no >= 0 && !this->pose_reached_flag){
        this->execute_pose();
    }
    delay(5); //sleep for 5 ms after each run
}

void RobotController::read_serial(){
    static byte ndx = 0;
    bool end_msg = false;
    char endMarker = ';';
    char rc;
    short new_gait = -1;
    short new_pose = -1;
    
    while (this->stream->available() > 0 && (!end_msg)) {
        rc = this->stream->read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0';
            ndx = 0;
            end_msg = true;
            if (DEBUG){
                this->stream->println(receivedChars);
            }
        }
    }
    // clear input buffer after ;
    while (this->stream->available() > 0) {
        this->stream->read();
    }
    end_msg = false;
    if (receivedChars[0] == 'v'){
        this->change_v_flag = true;
        this->velocity_setting = short(atoi(&receivedChars[1]));
    }
    if (receivedChars[0] == 'g'){
        this->balance_flag = false;
        this->dance_flag = false;
        if (receivedChars[1] == 'm'){
            if (receivedChars[2] == 'f'){
                new_gait = 0;
            }
            else if (receivedChars[2] == 'b'){
                new_gait = 1;
            }
            else if (receivedChars[2] == 'r'){
                new_gait = 2;
            }
            else if (receivedChars[2] == 'l'){
                new_gait = 3;
            }
        }
        else if (receivedChars[1] == 't'){
            if (receivedChars[2] == 'l'){
                new_gait = 4;
            }
            if (receivedChars[2] == 'r'){
                new_gait = 5;
            }
        }
    }

    if (receivedChars[0] == 'p'){
        this->balance_flag = false;
        this->dance_flag = false;
        if (receivedChars[1] == 'n'){
            new_pose = 0;
        }
        else if (receivedChars[1] == 'l'){
            if (receivedChars[2] == 'u'){
                new_pose = 1;
            }
            if (receivedChars[2] == 'd'){
                new_pose = 2;
            }
            if (receivedChars[2] == 'r'){
                new_pose = 3;
            }
            if (receivedChars[2] == 'l'){
                new_pose = 4;
            }
            if (receivedChars[2] == 'o'){
                new_pose = 6;
            }
        }
        else if (receivedChars[1] == 'h'){
            new_pose = 5;
        }
    }
    if (receivedChars[0] == 'd'){
        this->dance_flag = true;
        this->balance_flag = false;
        short bpm_value = short(atoi(&receivedChars[1]));
        if (bpm_value < 40){
            bpm_value = 40;
        }
        else if (bpm_value > 160){
            bpm_value = 160;
        }
        this->bpm_setting = bpm_value;
    }

    if (receivedChars[0] == 's'){
        if (this->current_gait_no >= 0) {
            this->initiate_reset_step();
        }
        else if (this->current_pose_no >= 0 || this->balance_flag) {
            this->pose_reached_flag = false;
        }
        this->current_pose_no = 0; // after stop the new pose is neutral!
        this->current_gait_no = -1;
        this->balance_flag = false;
        this->dance_flag = false;
    }

    if (receivedChars[0] == 'b'){
        this->initiate_reset_step();
        this->current_gait_no = -1;
        this->current_pose_no = -1;
        this->balance_flag = true;
        this->dance_flag = false;
    }

    // if current order changes:
    if (new_gait != this->current_gait_no && new_gait >=0){
        this->cur_sample = 0;
        this->cur_step = 0;
        this->initiate_reset_step();
        this->init_flag = true;
        this->current_gait_no = new_gait;
        this->current_pose_no = -1;
    }
    else if(new_pose != this->current_pose_no && new_pose >=0){
        this->cur_sample = 0;
        this->cur_step = 0;
        this->init_flag = false;
        this->pose_reached_flag = false;
        this->current_gait_no = -1;
        this->current_pose_no = new_pose;
        if (!(this->current_gait_no >= 0)){
            this->robot_model.pose_list[this->current_pose_no]->calc_pose_lists(short(2));
            if (DEBUG){
                this->stream->println(String("Setting pose to ") + this->robot_model.pose_list[new_pose]->get_name() + ".");
            }
        }
        else {
            this->initiate_reset_step();
        }
    }

    if (receivedChars[0] == 'c') {
        // Example expected input: "cp09,355;" or "cd10,-1;"
        char* comma = strchr(receivedChars, ',');
        if (comma) {
            *comma = '\0'; // split the string at comma for parsing
            short port_no = atoi(&receivedChars[2]);   // skip 'cp' or 'cd'
            short value = atoi(comma + 1);             // after the comma

            if (receivedChars[1] == 'p') { // pwm setting
                if (value > SERVOMAX) {
                    value = SERVOMAX;
                } 
                else if (value < SERVOMIN) {
                    value = SERVOMIN;
                }
                this->change_init_pwm(port_no, value);
                this->set_init_pwm();
            } 
            else if (receivedChars[1] == 'd') { // direction setting
                if (value > 1) {
                    value = 1;
                } 
                else if (value < -1) {
                    value = -1;
                }
                this->change_act_dir(port_no, value);
                this->set_init_pwm();
            }
        } 
        else if (DEBUG) {
            this->stream->println("Invalid format for 'c' command.");
        }
        this->pose_reached_flag = false;
    }
    for (short ii=0; ii<8; ++ii){
        receivedChars[ii] = '0';
    }  
}

void RobotController::write_serial(String mssg){
      this->stream->println(mssg);
    }

void RobotController::get_current_gait_no() { 
    char* name;
    if (0 < this->current_gait_no && this->current_gait_no < 6) {
        this->robot_model.gait_list[current_gait_no]->get_name(name);
        this->write_serial(String("current gait: ") + name);
    }
    else {
        this->write_serial(String("no gait currently running"));
    }
}

void RobotController::get_current_pose_no(){
    if (0 < this->current_pose_no && this->current_pose_no < 7) {
        this->write_serial(String("current pose: ") + this->robot_model.pose_list[current_pose_no]->get_name());
    }
    else {
        this->write_serial(String("no pose currently running"));
    }
}