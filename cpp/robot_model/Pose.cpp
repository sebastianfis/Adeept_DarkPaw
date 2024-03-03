#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

Pose::Pose(SpiderLeg* leg_list[4], float movement_goal[4][3], short n, String name = "") {
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
    if (n < 2) {this->n = 2;}
    else {this->n = n;}; // Ensure n is at least 2
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

void Pose::calc_pose_lists(float coord_list[][4][3], short coord_list_PWM[][4][3]) {
    for (short leg = 0; leg < 4 ; ++leg) {
        float coordinates[this->n][3];
        float start[3];
        leg_list[leg]->get_cur_pos(start);
        float target[] = {movement_goal[leg][0], 
                          movement_goal[leg][1],
                          movement_goal[leg][2]};
        generate_straight_line(start, target, coordinates, this->n);
        float angles[this->n][3];
        short PWM_values[this->n][3];
        for (short sample = 0; sample < this->n; ++sample) {          
          leg_list[leg]->calc_trajectory(coordinates, angles, this->n);
          leg_list[leg]->calc_PWM(angles, PWM_values, this->n);
          for (short ii = 0; ii < 3; ++ii) {
            coord_list[sample][leg][ii] = coordinates[sample][ii];
            coord_list_PWM[sample][leg][ii] = PWM_values[sample][ii];
          }
    }
  }
}

String Pose::get_name() {
  return this->name;
}
