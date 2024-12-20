#ifndef Pose_h
#define Pose_h
#include "Arduino.h"
#include "SpiderLeg.h"
#include "Gait.h"

// note: In order to be used for indexing and index creating n_samples has to be declared const!!!
class Pose {
  private:
    //const short n_samples;
    const char* name;
    short total_samples;
    float movement_goal[4][3];
    float coord_list[8][4][3];
    short pwm_list[8][4][3];
  public:
    SpiderLeg* leg_list[4];
    Pose(SpiderLeg* leg_list[4], const char name[3]);
    void init(float movement_goal[4][3]);
    void set_movement_goal(float movement_goal[4][3]);
    void get_movement_goal(float target[4][3]);
    void calc_pose_lists(short n_samples);
    short get_sample_no();
    float get_coordinate_from_list(short sample, short leg, short ii);
    short get_pwm_from_list(short sample, short leg, short ii);
    const char* get_name();
};
#endif