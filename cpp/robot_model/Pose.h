#ifndef Pose_h
#define Pose_h
#include "SpiderLeg.h"
#include "Gait.h"


class Pose {
  private:
    short n;
    String name;
    float movement_goal[4][3];
  public:
    SpiderLeg* leg_list[4];
    Pose(SpiderLeg* leg_list[4], float movement_goal[4][3], short n, String name);
    void set_movement_goal(float movement_goal[4][3]);
    void calc_pose_lists(float coord_list[][4][3], short coord_list_PWM[][4][3]);
    String get_name();
};
#endif