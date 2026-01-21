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


// --- Serial command buffer ---
const byte numChars = 10;  /**< Maximum length of incoming serial commands */
char receivedChars[numChars]; /**< Serial command buffer */

// --- Preferences storage for actuator parameters ---
Preferences preferences;

/**
 * @brief PWM pin mapping for each actuator on each leg
 * port_list[leg_no][actuator_no]
 */
const short port_list[4][3] = {{9, 8, 7},    /**< Leg 0 (RF) */
                               {15, 14, 13}, /**< Leg 1 (LF) */
                               {6, 5, 4},    /**< Leg 2 (RB) */
                               {12, 11, 10}};/**< Leg 3 (LB) */


/**
 * @brief Initial PWM values for each actuator
 */
const short init_pwm[4][3] = {{275, 280, 295},
                              {290, 290, 270},
                              {340, 315, 325},
                              {275, 305, 310}};

/**
 * @brief Actuator direction mapping (1 = normal, -1 = reversed)
 */
const short actuator_direction[4][3] = {{1, -1, 1},
                                        {-1, 1, -1},
                                        {1, 1, -1},
                                        {-1, -1, 1}};

/**
 * @brief Constructor for RobotController
 * @param leg_list Array of 4 SpiderLeg pointers
 * @param serial Pointer to HardwareSerial object for communication
 * @param pwm Pointer to Adafruit_PWMServoDriver for servos
 * @param mpu Pointer to Adafruit_MPU6050 sensor
 *
 * Initializes controller variables and hardware references.
 */
RobotController::RobotController(SpiderLeg *leg_list[4],
                                 HardwareSerial* serial,
                                 Adafruit_PWMServoDriver* pwm,
                                 Adafruit_MPU6050* mpu):
robot_model(leg_list) {
    // Save hardware references
    this->stream = serial;
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
    // Initialize control state variables
    this->current_gait_no = -1;       /**< No gait active initially */
    this->current_pose_no = -1;       /**< No pose active initially */
    this->cur_sample = 0;             /**< Current sample index in step */
    this->cur_step = 0;               /**< Current step index in gait/reset */
    this->bpm_setting = 80;           /**< Default dance BPM */
    this->velocity_setting = 100;     /**< Default gait speed percentage */
    this->change_v_flag = false;      /**< Flag to update velocity */
    this->init_flag = false;          /**< Flag for init gait */
    this->reset_flag = false;         /**< Flag for reset step execution */
    this->pose_reached_flag = false;  /**< Flag indicating pose reached */
    this->dance_flag = false;         /**< Flag indicating dance mode active */
    this->balance_flag = false;       /**< Flag indicating balance mode active */
    this->last_command_ms = 0;        /**< Initialize timekeeping since last command */
    this->COMMAND_TIMEOUT_MS = 1000;  /**< Timeout for communication failure detection */
    this->comms_lost = false;         /**< Flag indicating communication failure */
    this->controller_armed = false;   /**< Flag indicating armed state of the ESP for movement */
    this->last_controller_armed = false; /**< Previous controller armed state */
    this->last_balance_flag = false;    /**< Previous balanace flag state */
    this->last_dance_flag = false;      /**< Previous dance flag state */
    this->last_gait_no = -2;            /**< Previous gait no */
    this->last_pose_no = -2;            /**< Previous pose no */

    // Save hardware interfaces
    this->pwm = pwm;
    this->mpu = mpu;
}

/**
 * @brief Initialize robot controller, PWM driver, MPU sensor, and leg actuators
 *
 * Sets up serial communication, PWM frequency, actuator PWM values,
 * initializes MPU6050 and calculates initial pose of robot.
 */
void RobotController::init(){
    // Begin serial communication
    this->stream->begin(115200);
    // Calculate PWM update period in microseconds
    this->pwm_update_period = round(1e6/SERVO_FREQ);
    // Reset timers and control variables
    this->time_now = 0;
    this->sensor_timer = 0;
    this->last_theta_x = 0;
    this->last_theta_y = 0;
    this->integral_x = 0;
    this->integral_y = 0;
    // Initialize PWM driver
    this->pwm->begin();
    this->pwm->setOscillatorFrequency(24700000);
    this->pwm->setPWMFreq(SERVO_FREQ);
    if (DEBUG) {
        this->stream->println(String("PWM driver successfully initialized!"));
    }

    // Begin non-volatile storage for actuator settings
    preferences.begin("act_data", false);

    // Store default actuator PWM and direction values if not already stored
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

    // Set initial PWM for all actuators
    this->set_init_pwm();

    delay(100); // wait for hardware to stabilize
    // Initialize MPU6050 sensor
    this->mpu->begin();
    this->mpu->setGyroRange(MPU6050_RANGE_250_DEG);
    this->mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);
    this->evaluate_mpu_errors(); // compute initial sensor bias
    if (DEBUG) {
         this->stream->println(String("MPU6050 sensor successfully initialized!"));
    }
}

/**
 * @brief Initialize PWM values for all leg actuators using stored preferences
 *
 * Retrieves the stored initial PWM and actuator direction values and
 * configures all actuators of each leg accordingly. Also initializes
 * the robot model and executes the initial pose until reached.
 */
void RobotController::set_init_pwm(){
    for (short leg_no = 0; leg_no < 4; ++leg_no){
        // Initialize each actuator with stored PWM and direction
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
         this->stream->println(String("Leg actuators successfully initialized!"));
    }

    // Initialize robot model poses
    this->current_pose_no = 0;
    this->robot_model.init();

    // Execute initial pose until reached
    while(!this->pose_reached_flag){
        this->execute_pose();
    }
}

/**
 * @brief Change stored initial PWM value for a specific actuator port
 * @param port PWM port number
 * @param init_pwm_value New initial PWM value
 *
 * Updates the preferences for the actuator corresponding to the given port.
 */
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

/**
 * @brief Change stored actuator direction for a specific actuator port
 * @param port PWM port number
 * @param act_dir_value New direction value (-1 or 1)
 */
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

/**
 * @brief Evaluate and store MPU6050 sensor biases
 *
 * Collects 200 readings from MPU accelerometer and gyroscope to
 * calculate offsets for X and Y axes.
 */
void RobotController::evaluate_mpu_errors(){
    short ii =0;
    sensors_event_t a, g, temp;
    float AccErrorX = 0;
    float AccErrorY = 0;
    float GyroErrorX = 0;
    float GyroErrorY = 0;
    while (ii < 200){
        this->mpu->getEvent(&a, &g, &temp);
        // Sum accelerometer angles
        AccErrorX = AccErrorX + ((atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
        // Sum gyroscope readings
        GyroErrorX = GyroErrorX + g.gyro.x;
        GyroErrorY = GyroErrorY + g.gyro.y;
        ii++;
    }
    // Store average errors
    this->acc_x_error = AccErrorX/200;
    this->acc_y_error = AccErrorY/200;
    this->gyro_x_error = GyroErrorX/200;
    this->gyro_y_error = GyroErrorY/200;
}

/**
 * @brief Execute robot reset sequence step by step
 *
 * Iterates over reset steps and samples, updating leg positions and PWM.
 */
void RobotController::execute_reset(){
    float coord_value[3];
    short pwm_value[3];
    if ((micros() - this->time_now)>=this->pwm_update_period) {
        this->time_now = micros();
        // if (DEBUG){
        //     this->stream->println(String("Reset step no ") + this->cur_step + ", Sample no " + this->cur_sample);
        // }
        // Update each leg
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

/**
 * @brief Randomly selects and executes a dance pose at a specific BPM
 *
 * Determines a new pose randomly, ensuring it differs from the current pose,
 * resets the step/sample counters, and disables gait/balance modes.
 */
void RobotController::dance(){
    unsigned long dance_update = round(1e6/short(this->bpm_setting)*60);
    short new_pose = -1;
    if ((micros() - this->time_now)>= dance_update) {
        this->time_now = micros();
        if (this->current_gait_no >= 0){
            this->initiate_reset_step();
            this->current_gait_no = -1;
        }
        // Pick a random new pose different from current
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

/**
 * @brief Execute the currently active gait sequence step by step
 *
 * Updates each leg’s coordinates and PWM according to the gait list.
 * Increments sample and step counters and resets when the gait is complete.
 */
void RobotController::execute_gait(){
    float coord_value[3];
    short pwm_value[3];

    if ((micros() - this->time_now)>=this->pwm_update_period) {
        this->time_now = micros();
        // if (DEBUG){
        //     if (this->init_flag){
        //         this->stream->println(String("Init step no ") + this->cur_step + ", Sample no " + this->cur_sample);
        //     }
        //     else{
        //         this->stream->println(String("Step no ") + this->cur_step + ", Sample no " + this->cur_sample);
        //     }
        // }
        // Update each leg
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

/**
 * @brief Execute the currently active pose step by step
 *
 * Updates leg coordinates and PWM according to the pose list until the pose is reached.
 */
void RobotController::execute_pose(){
    float coord_value[3];
    short pwm_value[3];

    if (!this->pose_reached_flag){
        if ((micros() - this->time_now)>=this->pwm_update_period) {
            this->time_now = micros();
            // if (DEBUG){
            //     this->stream->println(String("Sample no ") + this->cur_sample);
            // }
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
                // if (DEBUG && this->current_pose_no >= 0){
                //     this->stream->println(String("Pose ") + this->robot_model.pose_list[this->current_pose_no]->get_name() + " reached!");
                // }
            }
        }
    }
}

/**
 * @brief Balance the robot using MPU6050 data and PI control
 *
 * Reads accelerometer and gyroscope data, applies a Kalman-like correction,
 * computes body tilt, and updates leg positions to maintain balance.
 */
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

    unsigned long time_elapsed = micros() - this->sensor_timer;
    if ( time_elapsed >= 2*this->pwm_update_period) {
        this->mpu->getEvent(&a, &g, &temp);
        // Calculate accelerometer angles and apply error correction
        accAngleX = ((atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI)) - this->acc_x_error;
        accAngleY = ((atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI)) - this->acc_y_error;
        // Kalman-like filtering
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
        //Update last values
        this->last_theta_y = this->cur_theta_y;
        this->last_theta_x = this->cur_theta_x;

        if (DEBUG){
            this->stream->println(String("Measured angles: Theta_x = ") + accAngleY + " deg, Theta_y = " + accAngleX + " deg");
        }
        this->sensor_timer = micros();
        // Calculate new leg positions
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

/**
 * @brief Initiates a reset step for the robot
 *
 * Calculates the reset step sequences and sets the reset flag.
 */
void RobotController::initiate_reset_step(){
    if (this->current_gait_no >= 0){
            this->cur_step = 0;
            this->cur_sample = 0;
            this->robot_model.calc_reset_step();
            this->reset_flag = true;
        }
}

/**
 * @brief Main loop of the robot controller
 *
 * Reads serial commands, updates velocity if needed, executes balance, dance,
 * reset, gait, or pose depending on current flags. Includes a 5 ms delay at the end.
 */
void RobotController::run(){

    this->read_serial();  // ALWAYS read serial

    if (controller_armed) {
        if (millis() - this->last_command_ms > this->COMMAND_TIMEOUT_MS) {
            if (!comms_lost) {
                comms_lost = true;

                this->initiate_reset_step();
                this->current_gait_no = -1;
                this->current_pose_no = 0;
                this->balance_flag = false;
                this->dance_flag = false;

                controller_armed = false; // disarm again

                if (DEBUG) {
                    this->stream->println("Controller lost, disarming");
                }
            }
            delay(5);
            return;
        }
    }

    // Update velocity if requested
    if (this->change_v_flag){
        this->robot_model.set_velocity(this->velocity_setting);
        if (DEBUG){
            this->stream->println(String("Set velocity to ") + this->velocity_setting + " %");
            }
        this->change_v_flag=false;
    }
    // Execute balance, dance, reset, gait, or pose
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

/**
 * @brief Reads serial input from the controller
 *
 * Parses commands for velocity, gait, pose, dance, stop, balance, and
 * PWM/direction configuration. Updates internal flags and robot state.
 */
void RobotController::read_serial(){
    static byte ndx = 0;
    bool end_msg = false;
    char endMarker = ';';
    char rc;
    short new_gait = -1;
    short new_pose = -1;

    // Read incoming characters until end marker
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
        }
    }

    // when end_msg == true
    if (end_msg) {
        this->last_command_ms = millis();
        this->comms_lost = false;
    }

    // clear input buffer after ;
    while (this->stream->available() > 0) {
        this->stream->read();
    }
    end_msg = false;
    // Process commands
    if (receivedChars[0] == 'i' && receivedChars[1] == '\0') {
        this->controller_armed = true;
        goto clear_buffer;
    }
    if (receivedChars[0] == 'h' && receivedChars[1] == '\0') {
        // heartbeat only, no side effects
        goto clear_buffer;
    }
    if (receivedChars[0] == 'v'){ // Change velocity
        this->change_v_flag = true;
        this->velocity_setting = short(atoi(&receivedChars[1]));
    }
    if (receivedChars[0] == 'g'){ // Select gait
        this->balance_flag = false;
        this->dance_flag = false;
        if (receivedChars[1] == 'm'){ // move gait
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
        else if (receivedChars[1] == 't'){ // turn gait
            if (receivedChars[2] == 'l'){
                new_gait = 4;
            }
            if (receivedChars[2] == 'r'){
                new_gait = 5;
            }
        }
    }

    if (receivedChars[0] == 'p'){ // Select pose
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
    if (receivedChars[0] == 'd'){ // Dance command
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

    if (receivedChars[0] == 's'){ // Stop command
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

    if (receivedChars[0] == 'b'){ // Balance command
        this->initiate_reset_step();
        this->current_gait_no = -1;
        this->current_pose_no = -1;
        this->balance_flag = true;
        this->dance_flag = false;
    }

    // Update gait or pose if changed
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
        }
        else {
            this->initiate_reset_step();
        }
    }

    // Change PWM or actuator direction
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
    // Clear buffer
    clear_buffer:
    for (short ii=0; ii<8; ++ii){
        receivedChars[ii] = '\0';
    }
    if (DEBUG) {
        if (this->controller_armed && !this->last_controller_armed) {
            this->stream->println("Controller armed");
        }

        if (this->balance_flag && !this->last_balance_flag) {
            this->stream->println("Balance mode active");
        }

        if (this->dance_flag && !this->last_dance_flag) {
            this->stream->println("Dance mode active");
        }

        if (this->current_gait_no != this->last_gait_no && this->current_gait_no >= 0) {
            this->stream->println(String("Gait set to ") + this->robot_model.gait_list[this->current_gait_no]->get_name());
        }

        if (this->current_pose_no != this->last_pose_no && this->current_pose_no >= 0) {
            this->stream->println(
                String("Pose set to ") +
                this->robot_model.pose_list[this->current_pose_no]->get_name()
            );
        }
    }
    this->last_controller_armed = this->controller_armed;
    this->last_balance_flag = this->balance_flag;
    this->last_dance_flag = this->dance_flag;
    this->last_gait_no = this->current_gait_no;
    this->last_pose_no = this->current_pose_no;
}

/**
 * @brief Sends a message over the serial stream
 *
 * @param mssg Message string to send
 */
void RobotController::write_serial(String mssg){
      this->stream->println(mssg);
    }

/**
 * @brief Prints the name of the currently active gait over serial
 */
void RobotController::get_current_gait_no() {
    char gait_name[3];
    if (0 < this->current_gait_no && this->current_gait_no < 6) {
        this->write_serial(String("current gait: ") + this->robot_model.gait_list[current_gait_no]->get_name());
    }
    else {
        this->write_serial(String("no gait currently running"));
    }
}

/**
 * @brief Prints the name of the currently active pose over serial
 */
void RobotController::get_current_pose_no(){
    if (0 < this->current_pose_no && this->current_pose_no < 7) {
        this->write_serial(String("current pose: ") + this->robot_model.pose_list[current_pose_no]->get_name());
    }
    else {
        this->write_serial(String("no pose currently running"));
    }
}