#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

const short n_pose_max = 8; // Set the max number of samples
const short n_pose_min = 2; // Set the min number of samples

Pose::Pose(SpiderLeg* leg_list[4], float movement_goal[4][3], char* name = "   "){
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
    this->name = name;
    set_movement_goal(movement_goal);
}

void Pose::set_movement_goal(float movement_goal[4][3]){
  for (short ii = 0; ii < 4 ; ++ii) {
      for (short jj = 0; jj < 3 ; ++jj) {
        this->movement_goal[ii][jj] = movement_goal[ii][jj];
      }
    }
}

void Pose::get_movement_goal(float target[4][3]){
  for (short ii = 0; ii < 4 ; ++ii) {
      for (short jj = 0; jj < 3 ; ++jj) {
        target[ii][jj] = this->movement_goal[ii][jj];
      }
    }
}

void Pose::calc_pose_lists(short n_samples) {
  if (n_samples < n_pose_min) {
    n_samples = n_pose_min;
  }
  else if (n_samples > n_pose_max) {
    n_samples = n_pose_max;
  }
  float start[3];
  float target[3];
  float coordinates[n_pose_max][3];
  for (short leg = 0; leg < 4 ; ++leg) {
    this->leg_list[leg]->get_cur_pos(start);
    target[0] = this->movement_goal[leg][0];
    target[1] = this->movement_goal[leg][1];
    target[2] = this->movement_goal[leg][2];
    generate_straight_line(start, target, coordinates, n_samples);
    write_data_to_lists(this->leg_list[leg], leg, coordinates, this->coord_list, this->pwm_list, n_samples);
  }
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

float Pose::get_coordinate_from_list(short sample, short leg, short ii) {
  return this->coord_list[sample][leg][ii];
}
    
short Pose::get_pwm_from_list(short sample, short leg, short ii) {
  return this->pwm_list[sample][leg][ii];
}

char* Pose::get_name() {
  return this->name;
}
