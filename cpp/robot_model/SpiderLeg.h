#ifndef SpiderLeg_h
#define SpiderLeg_h
#include "math.h"
#include "FourBarLinkage.h"

class SpiderLeg {
private:
    String name;
    short dir_x, dir_y;
    float init_x_f, init_y_f, init_z_f;
    float init_phi, cur_x_f, cur_y_f, cur_z_f;
    

public:
    FourBarLinkage actuator1, actuator2, actuator3;
    SpiderLeg(short dir_x, short dir_y, String name);
    void forward_transform(float phi_1, float phi_2, float phi_3, float* x_f, float* y_f, float* z_f);
    void backward_transform(float x_f, float y_f, float z_f, float* phi_1, float* phi_2, float* phi_3);
    void calc_trajectory(float coords[][3], float angles[][3], int list_length);
    void calc_PWM(float angles[][3], short pwm[][3], short list_length);
    void update_actuator_angles(float phi_1, float phi_2, float phi_3);
    void update_cur_pos(float phi_1, float phi_2, float phi_3);
    void update_cur_phi(float x_f, float y_f, float z_f);
    void get_actuator_angles(float angles[3]);
    String get_name();
};
#endif