#ifndef SpiderLeg_h
#define SpiderLeg_h
#include "math.h"
#include "FourBarLinkage.h"

class SpiderLeg {
private:
    std::string name;
    int dir_x, dir_y;
    double x_j, y_j;
    double r_g, z_g, l_gp, l_pf;
    double psi_0, xi_0, theta_0, theta_leg;
    double init_x_f, init_y_f, init_z_f;
    double init_phi, cur_x_f, cur_y_f, cur_z_f;
    FourBarLinkage actuator1, actuator2, actuator3;

public:
    SpiderLeg(double x_j, double y_j, int dir_x, int dir_y, double r_g, double z_g, double l_gp, 
              double l_pf, double psi_0, double xi_0, double theta_0, double theta_leg, std::string);
    void forward_transform(double phi_1, double phi_2, double phi_3, double x_f, double y_f, double z_f);
    void backward_transform(double x_f, double y_f, double z_f, double phi_1, double phi_2, double phi_3);
    void calc_trajectory(double coords[][3], double angles[][3], int list_length);
    void calc_PWM(double angles[][3], int pwm[][3], int list_length);
    void update_actuator_angles(double phi_1, double phi_2, double phi_3);
    void update_cur_pos(double phi_1, double phi_2, double phi_3);
    void update_cur_phi(double x_f, double y_f, double z_f);
    void get_actuator_angles(double angles[3]);
    std::string get_name();
};
#endif