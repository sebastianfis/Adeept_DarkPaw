#include "math.h"
//#include "Arduino.h"
#include "FourBarLinkage.h"

FourBarLinkage::FourBarLinkage(float l_sg, float l_sa, float l_ab, float l_gb, float phi_0) {
    this->l_sg = l_sg;
    this->l_sa = l_sa;
    this->l_ab = l_ab;
    this->l_gb = l_gb;
    this->phi_max = M_PI;
    this->phi_min = -M_PI;
    this->theta_max = M_PI;
    this->theta_min = -M_PI;
    this->cur_phi = phi_0 * M_PI / 180.0;
    this->cur_theta = calc_theta(cur_phi);
    this->phi_0 = phi_0 * M_PI / 180.0;
    this->pwm_0 = 330;
    this->actuator_direction = 1;
}

void FourBarLinkage::set_pwm_init(short pwm_value, short actuator_dir) {
    assert(actuator_dir == 1 || actuator_dir == -1 && "actuator_direction must be 1 or -1!");
    this->pwm_0 = pwm_value;
    this->actuator_direction = actuator_dir;
}

short FourBarLinkage::calc_PWM(float phi) {
    float pwm_value = this->pwm_0 + this->actuator_direction * (phi - this->phi_0) * 460 / M_PI;
    if (pwm_value < 100) {
        pwm_value = 100;
    } else if (pwm_value > 560) {
        pwm_value = 560;
    }
    return short(pwm_value);
}

float FourBarLinkage::calc_theta(float phi) {
    float l_ag = sqrt(this->l_sa * this->l_sa + this->l_sg * this->l_sg - 2 * this->l_sa * this->l_sg * cos(phi));
    assert((this->phi_min <= phi && phi <= this->phi_max) &&
    "Movement not possible, because l_ag > l_ab + l_gb");
    float v1 = constrain((this->l_sg * this->l_sg + l_ag * l_ag - this->l_sa * this->l_sa) /
                                    (2 * this->l_sg * l_ag), -1.0, 1.0);
    float theta_1 = acos(v1);
    float v2 = constrain((this->l_gb * this->l_gb + l_ag * l_ag - this->l_ab * this->l_ab) /
                                    (2 * this->l_gb * l_ag), -1.0, 1.0);
    float theta_2 = acos(v2);
    float theta = theta_1 + theta_2;
    return theta;
}

float FourBarLinkage::calc_phi(float theta) {
    float l_bs = sqrt(this->l_gb * this->l_gb + this->l_sg * this->l_sg - 2 * this->l_gb *this-> l_sg * cos(theta));
    assert((this->theta_min <= theta && theta <= this->theta_max) &&
          "Movement not possible, because l_bs > l_ab + l_sa");
    float v1 =  constrain((this->l_sg * this->l_sg + l_bs * l_bs - this->l_gb * this->l_gb) /
                                  (2 * this->l_sg * l_bs), -1.0, 1.0);
    float phi_1 = acos(v1);
    float v2 =  constrain((this->l_sa * this->l_sa + l_bs * l_bs - this->l_ab * this->l_ab) /
                                  (2 * l_sa * l_bs), -1.0, 1.0);
    double phi_2 = acos(v2);
    double phi = phi_1 + phi_2;
    return phi;
}

void FourBarLinkage::calc_limits(float phi_0) {
    float l_ag_max = this->l_ab + this->l_gb;
    float v1 = constrain((this->l_sa * this->l_sa + this->l_sg * this->l_sg - l_ag_max * l_ag_max) /
                                     (2 * this->l_sa * this->l_sg), -1.0, 1.0);
    float phi_max = acos(v1);
    if (this->phi_max > phi_0 + M_PI / 2) {
        this->phi_max = phi_0 + M_PI / 2;
    } else {
        this->phi_max = phi_max;
    }

    this->theta_min = calc_theta(phi_max);

    float l_bs_max = this->l_ab + this->l_sa;
    float v2 = constrain((this->l_gb * this->l_gb + this->l_sg * this->l_sg - l_bs_max * l_bs_max) /
                                       (2 * this->l_gb * this->l_sg), -1.0, 1.0);
    float theta_max = acos(v2);
    this->theta_max = theta_max;
    float phi_min = calc_phi(theta_max);
    if (this->phi_min < phi_0 - M_PI / 2) {
        this->phi_min = phi_0 - M_PI / 2;
    } else {
        this->phi_min = phi_min;
    }
}

float FourBarLinkage::get_cur_phi() {
  return this->cur_phi;
}

void FourBarLinkage::set_cur_phi(float phi) {
  this->cur_phi = phi;
}

void FourBarLinkage::set_cur_theta(float theta) {
  this->cur_theta = theta;
}

float FourBarLinkage::get_phi_0() {
    return this->phi_0;
}
