#ifndef RobotModel_h
#define RobotModel_h
#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

class RobotModel {
  private:
    float update_freq, step_height, z_max, z_min;
    float v_max_x, v_max_y, v_max_t, v_min_x, v_min_y, v_min_t;
    float reset_coord_list[5][8][4][3];
    short reset_pwm_list[5][8][4][3];
  public:
    SpiderLeg * leg_list[4];
    Gait move_forward, move_backward, move_right, move_left, turn_right, turn_left;
    Pose neutral, look_up, look_down, lean_right, lean_left, high, low;
    Gait * gait_list[6];
    Pose * pose_list[7];
    RobotModel(SpiderLeg* leg_list[4]);
    void init();
    void set_velocity(short percentage);
    void get_body_angles(float angles[2]);
    void calc_reset_step();
    void calc_leg_pos_from_body_angles(float movement_goal[4][3], float theta_x, float theta_y, float z_0);
    float get_reset_coord(short step, short sample, short leg, short ii);
    short get_reset_pwm(short step, short sample, short leg, short ii);
};

#endif