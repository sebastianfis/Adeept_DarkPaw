#ifndef Pose_h
#define Pose_h
#include "SpiderLeg.h"


class Pose {
  private:
    short n;
    String name;
    float movement_goal[4][3];
  public:
    SpiderLeg *leg_list[4];
    Pose(SpiderLeg *RFL, SpiderLeg *LFL, SpiderLeg *RBL, SpiderLeg *LBL, float movement_goal[4][3], String name, short n);
    void calc_pose_lists(float* angle_list[][4][3], short* coord_list_PWM[][4][3]);
    String get_name();
};
#endif