#include "SpiderLeg.h"
#include "Pose.h"

Pose::Pose(SpiderLeg *RFL, SpiderLeg *LFL, SpiderLeg *RBL, SpiderLeg *LBL, float movement_goal[4][3], String name, short n) {
    this->leg_list[0] = RFL;
    this->leg_list[1] = LFL;
    this->leg_list[2] = RBL;
    this->leg_list[3] = LBL;
    if (n < 2) {this->n = 2;}
    else {this->n = n;}; // Ensure n is at least 2
    this->name = name;
    for (short ii = 0; ii < 4 ; ++ii) {
      for (short jj = 0; jj < 3 ; ++jj) {
        this->movement_goal[ii][jj] = movement_goal[ii][jj];
      }
    }
}

void Pose::calc_pose_lists(float* angle_list[][4][3], short* coord_list_PWM[][4][3]) {
    // Assuming that Leg class has the necessary methods (calc_PWM, calc_trajectory)
    for (short leg = 0; leg < 4 ; ++leg) {
        float x[n], y[n], z[n];
        // Gait::generate_straight_line(
        //     {leg_list[leg].cur_x_f, leg_list[leg].cur_y_f, leg_list[leg].cur_z_f},
        //     {movement_goal[leg][0], movement_goal[leg][1], movement_goal[leg][2]},
        //     n, x, y, z
        // );
        // //complete method description here!!!
        // // Assuming Leg class has a method calc_PWM that takes trajectory and returns PWM values
        //leg_list[leg].calc_trajectory(float coords[][3], float angles[][3], int list_length) 
        //leg_list[leg].calc_PWM(angle_list[sample][leg], coord_list_PWM[sample][leg], this->n +1)

        // You can use leg_trajectory and leg_pwm as needed
    }
};
