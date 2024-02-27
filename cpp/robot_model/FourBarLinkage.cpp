#include "math.h"
#include "Arduino.h"
#include "FourBarLinkage.h"

FourBarLinkage::FourBarLinkage(double l_sg, double l_sa, double l_ab, double l_gb, double phi_0) {
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

void FourBarLinkage::set_pwm_init(double pwm_value, int actuator_dir) {
    assert(actuator_dir == 1 || actuator_dir == -1 && "actuator_direction must be 1 or -1!");
    this->pwm_0 = pwm_value;
    this->actuator_direction = actuator_dir;
}

int FourBarLinkage::calc_PWM(double phi) {
    double pwm_value = this->pwm_0 + this->actuator_direction * (phi - this->phi_0) * 460 / M_PI;
    if (pwm_value < 100) {
        pwm_value = 100;
    } else if (pwm_value > 560) {
        pwm_value = 560;
    }
    return int(pwm_value);
}

double FourBarLinkage::calc_theta(double phi) {
    double l_ag = sqrt(this->l_sa * this->l_sa + this->l_sg * this->l_sg - 2 * this->l_sa * this->l_sg * cos(phi));
    assert((this->phi_min <= phi && phi <= this->phi_max) &&
    "Movement not possible, because l_ag > l_ab + l_gb");
    double v1 = constrain((this->l_sg * this->l_sg + l_ag * l_ag - this->l_sa * this->l_sa) /
                                    (2 * this->l_sg * l_ag), -1.0, 1.0);
    double theta_1 = acos(v1);
    double v2 = constrain((this->l_gb * this->l_gb + l_ag * l_ag - this->l_ab * this->l_ab) /
                                    (2 * this->l_gb * l_ag), -1.0, 1.0);
    double theta_2 = acos(v2);
    double theta = theta_1 + theta_2;
    return theta;
}

double FourBarLinkage::calc_phi(double theta) {
    double l_bs = sqrt(this->l_gb * this->l_gb + this->l_sg * this->l_sg - 2 * this->l_gb *this-> l_sg * cos(theta));
    assert((this->theta_min <= theta && theta <= this->theta_max) &&
          "Movement not possible, because l_bs > l_ab + l_sa");
    double v1 =  constrain((this->l_sg * this->l_sg + l_bs * l_bs - this->l_gb * this->l_gb) /
                                  (2 * this->l_sg * l_bs), -1.0, 1.0);
    double phi_1 = acos(v1);
    double v2 =  constrain((this->l_sa * this->l_sa + l_bs * l_bs - this->l_ab * this->l_ab) /
                                  (2 * l_sa * l_bs), -1.0, 1.0);
    double phi_2 = acos(v2);
    double phi = phi_1 + phi_2;
    return phi;
}

void FourBarLinkage::calc_limits(double phi_0) {
    double l_ag_max = this->l_ab + this->l_gb;
    double v1 = constrain((this->l_sa * this->l_sa + this->l_sg * this->l_sg - l_ag_max * l_ag_max) /
                                     (2 * this->l_sa * this->l_sg), -1.0, 1.0);
    double phi_max = acos(v1);
    if (this->phi_max > phi_0 + M_PI / 2) {
        this->phi_max = phi_0 + M_PI / 2;
    } else {
        this->phi_max = phi_max;
    }

    this->theta_min = calc_theta(phi_max);

    double l_bs_max = this->l_ab + this->l_sa;
    double v2 = constrain((this->l_gb * this->l_gb + this->l_sg * this->l_sg - l_bs_max * l_bs_max) /
                                       (2 * this->l_gb * this->l_sg), -1.0, 1.0);
    double theta_max = acos(v2);
    this->theta_max = theta_max;
    double phi_min = calc_phi(theta_max);
    if (this->phi_min < phi_0 - M_PI / 2) {
        this->phi_min = phi_0 - M_PI / 2;
    } else {
        this->phi_min = phi_min;
    }
}

double FourBarLinkage::get_cur_phi() {
  return this->cur_phi;
}

void FourBarLinkage::set_cur_phi(double phi) {
  this->cur_phi = phi;
}

void FourBarLinkage::set_cur_theta(double theta) {
  this->cur_theta = theta;
}

double FourBarLinkage::get_phi_0() {
    return this->phi_0;
}

// Function to plot_current_state is not included as it involves matplotlib for visualization which is not
// part of standard C++ libraries. You might need to use a different library or implement it according to
// your visualization needs in C++.
