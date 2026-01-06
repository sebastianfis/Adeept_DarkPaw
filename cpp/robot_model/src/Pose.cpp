#include "SpiderLeg.h"
#include "Pose.h"

/**
 * @brief Maximum allowed number of samples for interpolating a pose
 */
const short n_pose_max = 8;
/**
 * @brief Minimum number of samples for interpolating a pose
 */
const short n_pose_min = 2;

/**
 * @brief Construct a Pose object
 *
 * Associates the Pose with a set of SpiderLeg objects and assigns a name.
 *
 * @param leg_list Array of 4 pointers to SpiderLeg objects (RF, LF, RB, LB)
 * @param name     Optional name for this pose
 */
Pose::Pose(SpiderLeg* leg_list[4], const char* name = "   "){
  this->name = name;
  this->leg_list[0] = leg_list[0];
  this->leg_list[1] = leg_list[1];
  this->leg_list[2] = leg_list[2];
  this->leg_list[3] = leg_list[3];

}

/**
 * @brief Initialize the pose
 *
 * Sets the target positions for all legs and generates the trajectory tables.
 *
 * @param movement_goal 4x3 array specifying the desired end-effector positions for each leg
 */
void Pose::init(float movement_goal[4][3]) {
    set_movement_goal(movement_goal);
    this->calc_pose_lists(n_pose_min);
}

/**
 * @brief Store a movement goal for all legs
 *
 * Copies a 4x3 array of target coordinates into the object's internal state.
 *
 * @param movement_goal 4x3 array of target foot positions
 */
void Pose::set_movement_goal(float movement_goal[4][3]){
  for (short ii = 0; ii < 4 ; ++ii) {
      for (short jj = 0; jj < 3 ; ++jj) {
        this->movement_goal[ii][jj] = movement_goal[ii][jj];
      }
    }
}

/**
 * @brief Retrieve the stored movement goal
 *
 * Copies the internal movement goal into an external array.
 *
 * @param target 4x3 array to receive the movement goal
 */
void Pose::get_movement_goal(float target[4][3]){
  for (short ii = 0; ii < 4 ; ++ii) {
      for (short jj = 0; jj < 3 ; ++jj) {
        target[ii][jj] = this->movement_goal[ii][jj];
      }
    }
}

/**
 * @brief Calculate interpolated Cartesian and PWM trajectories for the pose
 *
 * For each leg:
 * - Computes a straight-line path from the current foot position to the movement goal
 * - Converts the trajectory into actuator angles and PWM values
 * - Stores results in internal lookup tables
 *
 * @param n_samples Number of interpolated points along the trajectory
 */
void Pose::calc_pose_lists(short n_samples) {
  this->total_samples = n_samples;
  // Clamp sample number within allowed bounds
  if (n_samples < n_pose_min) {
    n_samples = n_pose_min;
  }
  else if (n_samples > n_pose_max) {
    n_samples = n_pose_max;
  }
  float start[3];                // Current foot position
  float target[3];               // Desired foot position
  float coordinates[n_samples][3]; // Cartesian trajectory
  float angles[n_samples][3];      // Actuator angles
  short PWM_values[n_samples][3];  // Servo PWM signals

  // Compute trajectories for all 4 legs
  for (short leg_no = 0; leg_no < 4 ; ++leg_no) {
    // Get current foot position
    this->leg_list[leg_no]->get_cur_pos(start);
    // Set target for this leg
    target[0] = this->movement_goal[leg_no][0];
    target[1] = this->movement_goal[leg_no][1];
    target[2] = this->movement_goal[leg_no][2];
    // Generate straight-line trajectory from start to target
    generate_straight_line(start, target, coordinates, n_samples);
    // Convert trajectory into actuator angles
    this->leg_list[leg_no]->calc_trajectory(coordinates, angles, n_samples);
    // Convert angles into PWM values for servo control
    this->leg_list[leg_no]->calc_PWM(angles, PWM_values, n_samples);
    // Store results in internal tables
    for (short sample = 0; sample < n_samples; ++sample) {
      for (short ii = 0; ii < 3; ++ii) {
        this->coord_list[sample][leg_no][ii] = coordinates[sample][ii];
        this->pwm_list[sample][leg_no][ii] = PWM_values[sample][ii];
      }
    }
  }
  // Fill unused samples with sentinel values (-1000 for coordinates, -1 for PWM)
  if (n_samples < n_pose_max) {
    for (short sample = n_samples; sample < n_pose_max; ++sample) {
      for (short leg = 0; leg < 4; ++leg) {
        for (short ii = 0; ii < 3; ++ii) {
          this->coord_list[sample][leg][ii] = -1000;
          this->pwm_list[sample][leg][ii] = -1;
        }
      }
    }
  }
}

/**
 * @brief Get a Cartesian coordinate from the pose table
 *
 * @param sample Sample index along the trajectory
 * @param leg    Leg index (0=RF, 1=LF, 2=RB, 3=LB)
 * @param ii     Coordinate index (0=x, 1=y, 2=z)
 * @return Requested Cartesian value
 */
float Pose::get_coordinate_from_list(short sample, short leg, short ii) {
  return this->coord_list[sample][leg][ii];
}

/**
 * @brief Get a PWM value from the pose table
 *
 * @param sample Sample index along the trajectory
 * @param leg    Leg index
 * @param ii     Servo index (0,1,2)
 * @return PWM value for this servo
 */
short Pose::get_pwm_from_list(short sample, short leg, short ii) {
  return this->pwm_list[sample][leg][ii];
}

/**
 * @brief Get the total number of samples for the pose trajectory
 *
 * @return Number of samples
 */
short Pose::get_sample_no() {
  return this->total_samples;
}

/**
 * @brief Get the name of the Pose
 *
 * @return Pointer to character string containing the name
 */
const char* Pose::get_name() {
  return this->name;
}
