#ifndef Pose_h
#define Pose_h
#include "Arduino.h"
#include "SpiderLeg.h"
#include "Gait.h"

/**
 * @class Pose
 * @brief Represents a static or transitional pose for a quadruped robot
 *
 * The Pose class stores target positions for all four legs of the robot and
 * generates interpolated trajectories for the feet along straight-line paths.
 * It also provides lookup tables for actuator PWM signals, making it easier
 * to execute a smooth transition between poses.
 *
 * Legs are indexed as:
 * 0 = Right Front (RF)
 * 1 = Left Front (LF)
 * 2 = Right Back (RB)
 * 3 = Left Back (LB)
 */
class Pose {
  private:
    /**
     * @brief Name of the pose (optional)
     */
    const char* name;

    /**
     * @brief Number of interpolated samples in the trajectory
     */
    short total_samples;

    /**
     * @brief Target foot positions for each leg
     *
     * 4 legs × 3 coordinates (x, y, z)
     */
    float movement_goal[4][3];

    /**
     * @brief Cartesian coordinates for each sample in the trajectory
     *
     * Indexed as [sample][leg][coordinate]
     * Coordinates: 0=x, 1=y, 2=z
     * Maximum samples: 8
     */
    float coord_list[8][4][3];

    /**
     * @brief Corresponding servo PWM values for each trajectory sample
     *
     * Indexed as [sample][leg][actuator]
     */
    short pwm_list[8][4][3];

  public:
    /**
     * @brief Array of pointers to the SpiderLeg objects controlled by this pose
     */
    SpiderLeg* leg_list[4];

    /**
     * @brief Construct a Pose object
     *
     * @param leg_list Array of 4 SpiderLeg pointers (RF, LF, RB, LB)
     * @param name     Optional name for this pose
     */
    Pose(SpiderLeg* leg_list[4], const char name[3]);

    /**
     * @brief Initialize the pose with a target movement
     *
     * Generates trajectory and PWM tables for smooth motion
     *
     * @param movement_goal 4x3 array specifying desired foot positions
     */
    void init(float movement_goal[4][3]);

    /**
     * @brief Set the target positions for all legs
     *
     * @param movement_goal 4x3 array of desired foot positions
     */
    void set_movement_goal(float movement_goal[4][3]);

    /**
     * @brief Get the stored movement goal
     *
     * @param target 4x3 array to receive the movement goal
     */
    void get_movement_goal(float target[4][3]);

    /**
     * @brief Calculate the interpolated Cartesian and PWM trajectories
     *
     * Generates a straight-line path from the current foot positions
     * to the movement_goal for each leg and stores the results in internal tables.
     *
     * @param n_samples Number of interpolated samples (must be ≤ 8)
     */
    void calc_pose_lists(short n_samples);

    /**
     * @brief Get the total number of trajectory samples
     *
     * @return Number of samples used for this pose
     */
    short get_sample_no();

    /**
     * @brief Retrieve a Cartesian coordinate from the trajectory table
     *
     * @param sample Sample index along the trajectory
     * @param leg    Leg index (0=RF, 1=LF, 2=RB, 3=LB)
     * @param ii     Coordinate index (0=x, 1=y, 2=z)
     * @return Requested coordinate value
     */
    float get_coordinate_from_list(short sample, short leg, short ii);

    /**
     * @brief Retrieve a PWM value from the trajectory table
     *
     * @param sample Sample index along the trajectory
     * @param leg    Leg index
     * @param ii     Actuator index (0,1,2)
     * @return PWM value
     */
    short get_pwm_from_list(short sample, short leg, short ii);

    /**
     * @brief Get the name of the Pose
     *
     * @return Pointer to the name string
     */
    const char* get_name();
};

#endif
