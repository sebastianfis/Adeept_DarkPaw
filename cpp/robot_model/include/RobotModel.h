#ifndef RobotModel_h
#define RobotModel_h
#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

/**
 * @class RobotModel
 * @brief Represents a full quadruped robot, combining legs, gaits, and poses
 *
 * This class manages:
 * - Four SpiderLeg objects representing the robot's legs
 * - Predefined gaits (forward, backward, lateral, turning)
 * - Predefined poses (neutral, looking up/down, leaning, high/low)
 * - Trajectory planning for resetting the robot to initial positions
 * - Body orientation calculations and velocity scaling
 */
class RobotModel {
  private:
    // --- Robot motion parameters ---
    float update_freq;   // Frequency of updates in Hz
    float step_height;   // Default height of a foot step
    float z_max;         // Maximum robot body height
    float z_min;         // Minimum robot body height
    // --- Maximum and minimum velocity limits for different motion axes ---
    float v_max_x, v_max_y, v_max_t;  // Max velocity along X, Y, and turning
    float v_min_x, v_min_y, v_min_t;  // Min velocity along X, Y, and turning
    // --- Internal reset trajectory storage ---
    float reset_coord_list[5][8][4][3];  // Stores reset positions for each leg [step][sample][leg][xyz]
    short reset_pwm_list[5][8][4][3];    // Stores corresponding PWM values for actuators
  public:
    // --- Pointers to the robot's legs ---
    SpiderLeg * leg_list[4];
    // --- Gait objects for different robot movements ---
    Gait move_forward, move_backward, move_right, move_left, turn_right, turn_left;
    // --- Pose objects for predefined robot body configurations ---
    Pose neutral, look_up, look_down, lean_right, lean_left, high, low;
    // --- Arrays of pointers to gaits and poses for easier iteration/access ---
    Gait * gait_list[6];
    Pose * pose_list[7];
    /**
     * @brief Construct the RobotModel
     * @param leg_list Array of 4 SpiderLeg pointers
     *
     * Initializes gaits, poses, and stores leg references
     */
    RobotModel(SpiderLeg* leg_list[4]);
    /**
     * @brief Initialize robot parameters and default gaits/poses
     *
     * Sets update frequency, step height, max/min heights,
     * initializes gaits with step lengths and velocities, and
     * initializes standard poses using body angles
     */
    void init();
    /**
     * @brief Scale velocity of all gaits by a percentage
     * @param percentage 0-100 scale for velocity
     */
    void set_velocity(short percentage);
    /**
     * @brief Calculate approximate body pitch and roll angles
     * @param angles Output array {pitch, roll} in degrees
     */
    void get_body_angles(float angles[2]);
    /**
     * @brief Generate reset trajectory to bring legs back to initial positions
     *
     * Populates internal reset_coord_list and reset_pwm_list arrays
     */
    void calc_reset_step();
    /**
     * @brief Calculate target leg positions based on body orientation
     * @param movement_goal Output array for leg positions [4][3]
     * @param theta_x Pitch in degrees
     * @param theta_y Roll in degrees
     * @param z_0 Optional desired body height; if 0, uses average leg height
     */
    void calc_leg_pos_from_body_angles(float movement_goal[4][3], float theta_x, float theta_y, float z_0);
    /**
     * @brief Retrieve reset trajectory coordinate
     * @param step Step index in reset sequence
     * @param sample Sample index for interpolation
     * @param leg Leg index (0-3)
     * @param ii Coordinate index (0=x, 1=y, 2=z)
     * @return float Coordinate value
     */
    float get_reset_coord(short step, short sample, short leg, short ii);
    /**
     * @brief Retrieve reset trajectory PWM value
     * @param step Step index in reset sequence
     * @param sample Sample index for interpolation
     * @param leg Leg index (0-3)
     * @param ii Actuator index (0-2)
     * @return short PWM value
     */
    short get_reset_pwm(short step, short sample, short leg, short ii);
};

#endif
