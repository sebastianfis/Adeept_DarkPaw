#ifndef RobotController_h
#define RobotController_h
#include "Arduino.h"
#include "math.h"
#include "SpiderLeg.h"
#include "RobotModel.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

/**
 * @class RobotController
 * @brief Main class to control the spider robot.
 *
 * Handles communication, gait and pose execution, balance, dance, and actuator initialization.
 */
class RobotController {
  private:
  short current_gait_no;    ///< Index of the currently active gait
  short current_pose_no;    ///< Index of the currently active pose
  short cur_sample;         ///< Current sample within a gait or pose
  short cur_step;           ///< Current step within a gait or reset sequence
  short velocity_setting;   ///< Requested velocity in percentage
  short bpm_setting;        ///< Dance BPM setting
  float cur_theta_x;        ///< Current body angle x (from sensor)
  float cur_theta_y;        ///< Current body angle y (from sensor)
  float last_theta_x;       ///< Last measured x angle
  float last_theta_y;       ///< Last measured y angle
  float acc_x_error;        ///< Accelerometer x offset error
  float acc_y_error;        ///< Accelerometer y offset error
  float gyro_x_error;       ///< Gyroscope x offset error
  float gyro_y_error;       ///< Gyroscope y offset error
  float integral_x;         ///< Integral term for x-axis PI control
  float integral_y;         ///< Integral term for y-axis PI control
  bool change_v_flag;       ///< Flag to indicate velocity change requested
  bool init_flag;           ///< Flag to indicate gait initialization
  bool reset_flag;          ///< Flag to indicate reset step in progress
  bool pose_reached_flag;   ///< Flag to indicate pose reached
  bool dance_flag;          ///< Flag to indicate dance mode active
  bool balance_flag;        ///< Flag to indicate balance mode active
  unsigned long time_now;          ///< Timer for PWM updates
  unsigned long pwm_update_period; ///< PWM update period in microseconds
  unsigned long sensor_timer;      ///< Timer for sensor reading updates
  unsigned long last_command_ms;          ///< Initialize timekeeping since last command
  unsigned long COMMAND_TIMEOUT_MS;       ///< Timeout for communication failure detection
  bool comms_lost;                        ///< Flag indicating communication failure
  bool controller_armed;                  ///< Flag indicating armed state of the ESP for movement
  bool last_controller_armed;     ///< Previous controller armed state
  bool last_balance_flag;         ///< Previous balanace flag state
  bool last_dance_flag;           ///< Previous dance flag state
  short last_gait_no;             ///< Previous gait no
  short last_pose_no;             ///< Previous pose no

  public:
  HardwareSerial* stream;          ///< Serial interface pointer
  RobotModel robot_model;          ///< Robot kinematic and gait/pose model
  SpiderLeg* leg_list[4];          ///< Array of pointers to robot legs
  Adafruit_PWMServoDriver* pwm;    ///< PWM driver for actuators
  Adafruit_MPU6050* mpu;           ///< MPU6050 IMU sensor pointer
  /**
   * @brief Constructor for RobotController
   * @param leg_list Array of 4 SpiderLeg pointers
   * @param serial Serial interface pointer
   * @param pwm PWM driver pointer
   * @param mpu MPU6050 sensor pointer
   */
  RobotController(SpiderLeg* leg_list[4], HardwareSerial* serial, Adafruit_PWMServoDriver* pwm, Adafruit_MPU6050* mpu);
  /**
   * @brief Initializes serial, PWM, sensors, and robot model
   */
  void init();
  /**
   * @brief Initializes actuators with stored or default PWM values
   */
  void set_init_pwm();
  /**
   * @brief Updates the initial PWM value for a specific actuator port
   * @param port Actuator port number
   * @param init_pwm PWM value to store
   */
  void change_init_pwm(short port, short init_pwm);
  /**
   * @brief Updates actuator direction for a specific port
   * @param port Actuator port number
   * @param act_dir Direction value (-1 or 1)
   */
  void change_act_dir(short port, short act_dir);
  /**
   * @brief Main loop to update robot state
   *
   * Executes serial commands, gait, pose, dance, balance, and reset steps.
   */
  void run();
  /**
   * @brief Reads and parses serial commands from controller
   */
  void read_serial();
  /**
   * @brief Sends a message over the serial stream
   * @param mssg Message to send
   */
  void write_serial(String mssg);
  /**
   * @brief Prints the currently active gait over serial
   */
  void get_current_gait_no();
  /**
   * @brief Prints the currently active pose over serial
   */
  void get_current_pose_no();
  /**
   * @brief Executes a reset step sequence
   */
  void execute_reset();
  /**
   * @brief Randomly selects and executes a dance pose
   */
  void dance();
  /**
   * @brief Executes the current gait sequence step
   */
  void execute_gait();
  /**
   * @brief Executes the current pose sequence step
   */
  void execute_pose();
  /**
   * @brief Updates robot legs based on IMU feedback to maintain balance
   */
  void balance();
  /**
   * @brief Measures and stores the MPU sensor offsets/errors
   */
  void evaluate_mpu_errors();
  /**
   * @brief Initiates a reset step before gait or pose changes
   */
  void initiate_reset_step();
};

#endif