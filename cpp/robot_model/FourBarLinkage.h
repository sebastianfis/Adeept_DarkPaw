#ifndef FourBarLinkage_h
#define FourBarLinkage_h
#include "math.h"
#include "Arduino.h"

class FourBarLinkage {
private:
    float l_sg, l_sa, l_ab, l_gb;
    float phi_max, phi_min, theta_max, theta_min;
    float cur_phi, cur_theta, phi_0;
    short actuator_direction, pwm_0;

public:
    FourBarLinkage(float l_sg, float l_sa, float l_ab, float l_gb, float phi_0);
    void set_pwm_init(short pwm_value, short actuator_direction);
    short calc_PWM(float phi);
    float calc_theta(float phi);
    float calc_phi(float theta);
    void calc_limits(float phi_0);
    float get_cur_phi();
    void set_cur_phi(float phi);
    void set_cur_theta(float theta);
    float get_phi_0();
};
#endif