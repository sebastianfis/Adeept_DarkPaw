#ifndef FourBarLinkage_h
#define FourBarLinkage_h
#include "math.h"
#include "Arduino.h"

class FourBarLinkage {
private:
    double l_sg, l_sa, l_ab, l_gb;
    double phi_max, phi_min, theta_max, theta_min;
    double cur_phi, cur_theta, phi_0;
    int actuator_direction, pwm_0;

public:
    FourBarLinkage(double l_sg, double l_sa, double l_ab, double l_gb, double phi_0);
    void set_pwm_init(double pwm_value, int actuator_direction);
    int calc_PWM(double phi);
    double calc_theta(double phi);
    double calc_phi(double theta);
    void calc_limits(double phi_0);
    double get_cur_phi();
    void set_cur_phi(double phi);
    void set_cur_theta(double theta);
    double get_phi_0();
};
#endif