#ifndef SpiderLeg_h
#define SpiderLeg_h
#include "math.h"
#include "FourBarLinkage.h"

class SpiderLeg {
private:
    std::string name;
    int dir_x, dir_y;
    double init_x_f, init_y_f, init_z_f;
    double init_phi, cur_x_f, cur_y_f, cur_z_f;
    

public:
    FourBarLinkage actuator1, actuator2, actuator3;
    SpiderLeg(int dir_x, int dir_y, std::string name);
    void forward_transform(double phi_1, double phi_2, double phi_3, double* x_f, double* y_f, double* z_f);
    void backward_transform(double x_f, double y_f, double z_f, double* phi_1, double* phi_2, double* phi_3);
    void calc_trajectory(double coords[][3], double angles[][3], int list_length);
    void calc_PWM(double angles[][3], int pwm[][3], int list_length);
    void update_actuator_angles(double phi_1, double phi_2, double phi_3);
    void update_cur_pos(double phi_1, double phi_2, double phi_3);
    void update_cur_phi(double x_f, double y_f, double z_f);
    void get_actuator_angles(double angles[3]);
    std::string get_name();
};
#endif