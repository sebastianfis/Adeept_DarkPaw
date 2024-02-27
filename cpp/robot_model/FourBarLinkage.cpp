#include <iostream>
#include <cmath>

class FourBarLinkage {
private:
    double l_sg, l_sa, l_ab, l_gb;
    double phi_max, phi_min, theta_max, theta_min;
    double cur_phi, cur_theta, phi_0;
    int actuator_direction, pwm_0;

public:
    FourBarLinkage(double l_sg, double l_sa, double l_ab, double l_gb, double phi_0 = 90) {
        this->l_sg = l_sg;
        this->l_sa = l_sa;
        this->l_ab = l_ab;
        this->l_gb = l_gb;
        this->phi_max = M_PI;
        this->phi_min = -M_PI;
        this->theta_max = M_PI;
        this->theta_min = -M_PI;
        this->cur_phi = phi_0 * M_PI / 180.0;
        this->calc_limits(this->cur_phi);
        this->cur_theta = this->calc_theta(this->cur_phi);
        this->phi_0 = this->cur_phi;
        this->pwm_0 = 330;
        this->actuator_direction = 1;
    }

    double limit_values(double x, double a, double b){
      if (x < a){
          return a;
        }
      else if (x > b){
          return b;
        }
      else { 
        return x;
      }
    }

    void set_pwm_init(double pwm_value, int actuator_direction) {
        assert(actuator_direction == 1 || actuator_direction == -1 && "actuator_direction must be 1 or -1!");
        this->pwm_0 = pwm_value;
        this->actuator_direction = actuator_direction;
    }

    int calc_PWM(double phi) {
        double pwm_value = this->pwm_0 + this->actuator_direction * (phi - this->phi_0) * 460 / M_PI;
        if (pwm_value < 100) {
            pwm_value = 100;
        } else if (pwm_value > 560) {
            pwm_value = 560;
        }
        return int(pwm_value);
    }

    double calc_theta(double phi) {
        double l_ag = sqrt(this->l_sa * this->l_sa + this->l_sg * this->l_sg - 2 * this->l_sa * this->l_sg * cos(phi));
        assert(this->phi_min <= phi && phi <= this->phi_max &&
        "Movement not possible, because l_ag > l_ab + l_gb");
        double theta_1 = acos(limit_values((this->l_sg * this->l_sg + l_ag * l_ag - this->l_sa * this->l_sa) /
                                        (2 * this->l_sg * l_ag), -1.0, 1.0));
        double theta_2 = acos(limit_values((this->l_gb * this->l_gb + l_ag * l_ag - this->l_ab * this->l_ab) /
                                        (2 * this->l_gb * l_ag), -1.0, 1.0));
        double theta = theta_1 + theta_2;
        return theta;
    }

    double calc_phi(double theta) {
        double l_bs = sqrt(this->l_gb * this->l_gb + this->l_sg * this->l_sg - 2 * this->l_gb * this->l_sg * cos(theta));
        assert(this->theta_min <= theta && theta <= this->theta_max &&
               "Movement not possible, because l_bs > l_ab + l_sa");
        double phi_1 = acos(limit_values((this->l_sg * this->l_sg + l_bs * l_bs - this->l_gb * this->l_gb) /
                                      (2 * this->l_sg * l_bs), -1.0, 1.0));
        double phi_2 = acos(limit_values((this->l_sa * this->l_sa + l_bs * l_bs - this->l_ab * this->l_ab) /
                                      (2 * this->l_sa * l_bs), -1.0, 1.0));
        double phi = phi_1 + phi_2;
        return phi;
    }

    void calc_limits(double phi_0) {
        double l_ag_max = this->l_ab + this->l_gb;
        double phi_max = acos(limit_values((this->l_sa * this->l_sa + this->l_sg * this->l_sg - l_ag_max * l_ag_max) /
                                         (2 * this->l_sa * this->l_sg), -1.0, 1.0));
        if (phi_max > phi_0 + M_PI / 2) {
            this->phi_max = phi_0 + M_PI / 2;
        } else {
            this->phi_max = phi_max;
        }

        this->theta_min = this->calc_theta(this->phi_max);

        double l_bs_max = this->l_ab + this->l_sa;
        double theta_max = acos(limit_values((this->l_gb * this->l_gb + this->l_sg * this->l_sg - l_bs_max * l_bs_max) /
                                           (2 * this->l_gb * this->l_sg), -1.0, 1.0));
        this->theta_max = theta_max;
        double phi_min = this->calc_phi(this->theta_max);
        if (phi_min < phi_0 - M_PI / 2) {
            this->phi_min = phi_0 - M_PI / 2;
        } else {
            this->phi_min = phi_min;
        }
    }

    double get_cur_phi() {
      return this->cur_phi;
    }

    void set_cur_phi(double phi) {
      this->cur_phi = phi;
    }

    void set_cur_theta(double theta) {
      this->cur_theta = theta;
    }

    double get_phi_0() {
        return this->phi_0;
    }

    // Function to plot_current_state is not included as it involves matplotlib for visualization which is not
    // part of standard C++ libraries. You might need to use a different library or implement it according to
    // your visualization needs in C++.
};