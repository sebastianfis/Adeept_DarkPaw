#include <cmath>
#include <iostream>

class FourBarLinkage {
public:
    FourBarLinkage(double l_sg, double l_sa, double l_ab, double l_gb, double phi_0 = 90.0): 
        l_sg(l_sg), 
        l_sa(l_sa), 
        l_ab(l_ab), 
        l_gb(l_gb), 
        phi_0(phi_0) {
        phi_max = 2 * M_PI;
        phi_min = -2 * M_PI;
        theta_max = 2 * M_PI;
        theta_min = -2 * M_PI;
        calc_limits(deg2rad(phi_0));
        cur_phi = deg2rad(phi_0);
        cur_theta = calc_theta(cur_phi);
        this->phi_0 = deg2rad(phi_0);
        pwm_0 = 300;
        actuator_direction = 1;
    }

    void set_pwm_init(int pwm_value, int actuator_direction) {
        assert(actuator_direction == 1 || actuator_direction == -1);
        pwm_0 = pwm_value;
        this->actuator_direction = actuator_direction;
    }

    int calc_PWM(double phi) {
        double pwm_value = pwm_0 + actuator_direction * (phi - phi_0) * 400 / M_PI;
        if (pwm_value < 100)
            pwm_value = 100;
        else if (pwm_value > 500)
            pwm_value = 500;
        return static_cast<int>(pwm_value);
    }

    double calc_theta(double phi) {
        double l_ag = sqrt(l_sa * l_sa + l_sg * l_sg - 2 * l_sa * l_sg * cos(phi));
        assert(phi_min <= phi && phi <= phi_max && "Movement not possible!");
        double theta_1 = acos((l_sg * l_sg + l_ag * l_ag - l_sa * l_sa) / (2 * l_sg * l_ag));
        double theta_2 = acos((l_gb * l_gb + l_ag * l_ag - l_ab * l_ab) / (2 * l_gb * l_ag));
        double theta = theta_1 + theta_2;
        return theta;
    }

    double calc_phi(double theta) {
        double l_bs = sqrt(l_gb * l_gb + l_sg * l_sg - 2 * l_gb * l_sg * cos(theta));
        assert(theta_min <= theta && theta <= theta_max && "Movement not possible!");
        double phi_1 = acos((l_sg * l_sg + l_bs * l_bs - l_gb * l_gb) / (2 * l_sg * l_bs));
        double phi_2 = acos((l_sa * l_sa + l_bs * l_bs - l_ab * l_ab) / (2 * l_sa * l_bs));
        double phi = phi_1 + phi_2;
        return phi;
    }

    void calc_limits(double phi_0) {
        double l_ag_max = l_ab + l_gb;
        double phi_max = acos((l_sa * l_sa + l_sg * l_sg - l_ag_max * l_ag_max) / (2 * l_sa * l_sg));
        if (std::isnan(phi_max))
            phi_max = M_PI;
        if (phi_max > phi_0 + M_PI / 2)
            this->phi_max = phi_0 + M_PI / 2;
        else
            this->phi_max = phi_max;

        theta_min = calc_theta(this->phi_max);

        double l_bs_max = l_ab + l_sa;
        double theta_max = acos((l_gb * l_gb + l_sg * l_sg - l_bs_max * l_bs_max) / (2 * l_gb * l_sg));
        if (std::isnan(theta_max))
            theta_max = M_PI;
        this->theta_max = theta_max;

        double phi_min = calc_phi(theta_max);
        if (std::isnan(phi_min))
            phi_min = 0;

        if (phi_min < phi_0 - M_PI / 2)
            this->phi_min = phi_0 - M_PI / 2;
        else
            this->phi_min = phi_min;
    }

    double get_cur_phi() {
      return cur_phi;
    }

    void set_cur_phi(double phi) {
      cur_phi=phi;
    }

    double get_cur_theta() {
      return cur_theta;
    }

    void set_cur_theta(double theta) {
      cur_theta=theta;
    }

    // Add the remaining methods for plotting if needed...

private:
    double l_sg, l_sa, l_ab, l_gb;
    double phi_max, phi_min, theta_max, theta_min;
    double cur_phi, cur_theta, phi_0;
    int pwm_0, actuator_direction;

    double deg2rad(double degrees) {
        return degrees * M_PI / 180.0;
    }

    double rad2deg(double radians) {
        return radians * 180.0 / M_PI;
    }
};
