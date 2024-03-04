#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

Pose::Pose(SpiderLeg* leg_list[4], float movement_goal[4][3], short n, char* name = "   "): n_samples(n) {
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

void Pose::calc_pose_lists(float coord_list[][4][3], short coord_list_PWM[][4][3]) {
  float coordinates[this->n_samples][3];
  float angles[this->n_samples][3];
  short PWM_values[this->n_samples][3];
  float start[3];
  float target[3];
    for (short leg = 0; leg < 4 ; ++leg) {
        leg_list[leg]->get_cur_pos(start);
        target[0] = this->movement_goal[leg][0];
        target[1] = this->movement_goal[leg][1];
        target[2] = this->movement_goal[leg][2];
        generate_straight_line(start, target, coordinates, this->n_samples);
        
        for (short sample = 0; sample < this->n_samples; ++sample) {          
          leg_list[leg]->calc_trajectory(coordinates, angles, this->n_samples);
          leg_list[leg]->calc_PWM(angles, PWM_values, this->n_samples);
          for (short ii = 0; ii < 3; ++ii) {
            coord_list[sample][leg][ii] = coordinates[sample][ii];
            coord_list_PWM[sample][leg][ii] = PWM_values[sample][ii];
          }
        
    }
  }
}

char* Pose::get_name() {
  return this->name;
}
