#include <cmath>
#include <iostream>
#include "FourBarLinkage.cpp"


struct vector3 {
    double x;
    double y;
    double z;
};

struct int3 {
    int x;
    int y;
    int z;
};

class SpiderLeg {
public:
    std::string name;
    FourBarLinkage actuator1, actuator2, actuator3;
    double x_j, y_j, dir_x, dir_y;
    double r_g, z_g, l_gp, l_pf;
    double psi_0, xi_0, theta_0, theta_leg;
    double init_x_f, init_y_f, init_z_f;
    double init_phi, cur_x_f, cur_y_f, cur_z_f;

    SpiderLeg(double x_j = 43.5, double y_j = 42, double dir_x = 1, double dir_y = 1, const std::string& name = "",
              double r_g = 66.5, double z_g = -23, double l_gp = 30.5, double l_pf = 78.2,
              double psi_0 = 131.5, double xi_0 = 147.5, double theta_0 = 90 - 9.5, double theta_leg = 33.7)
        : name(name), actuator1(42.5, 14.5, 38, 27.8, 90), actuator2(35.6, 14.5, 35.6, 25.6, 99.5),
          actuator3(35.6, 14.5, 26, 38.5, 99.5), x_j(x_j), y_j(y_j), dir_x(dir_x), dir_y(dir_y),
          r_g(r_g), z_g(z_g), l_gp(l_gp), l_pf(l_pf), psi_0(psi_0), xi_0(xi_0), theta_0(theta_0), theta_leg(theta_leg),
          init_x_f(0), init_y_f(0), init_z_f(0), init_phi(0), cur_x_f(0), cur_y_f(0), cur_z_f(0) {
          forward_transform(actuator1.get_cur_phi(), actuator2.get_cur_phi(), actuator3.get_cur_phi(), init_x_f, init_y_f, init_z_f);
        init_phi = atan2(init_x_f, init_y_f);
        cur_x_f, cur_y_f, cur_z_f = init_x_f, init_y_f, init_z_f;
    }

    void forward_transform(double phi_1, double phi_2, double phi_3, double x_f, double y_f, double z_f) {
        double theta_1 = actuator1.calc_theta(phi_1);
        double theta_2 = actuator2.calc_theta(phi_2);
        double theta_3 = actuator3.calc_theta(phi_3);
        double r_f = r_g + l_gp * cos(theta_2 - theta_0) + l_pf * cos(xi_0 - psi_0 + theta_3);
        x_f = z_g - l_gp * sin(theta_2 - theta_0) + l_pf * sin(xi_0 - psi_0 + theta_3);
        y_f = x_j - dir_x * r_f * cos(theta_1 + theta_leg);
        z_f = y_j + dir_y * r_f * sin(theta_1 + theta_leg);
    }

    void backward_transform(double x_f, double y_f, double z_f, double phi_1, double phi_2, double phi_3) {
        double r_f = sqrt((x_f - x_j) * (x_f - x_j) + (y_f - y_j) * (y_f - y_j));
        double theta_1 = acos((x_j - x_f) / dir_x / r_f) - theta_leg;
        phi_1 = actuator1.calc_phi(theta_1);
        double vec_length = (r_f - r_g) * (r_f - r_g) + (z_f - z_g) * (z_f - z_g);
        double theta_2 =
            acos((l_gp * l_gp - l_pf * l_pf + vec_length) / (2 * l_gp * sqrt(vec_length))) -
            atan2(z_f - z_g, r_f - r_g) + theta_0;
        phi_2 = actuator2.calc_phi(theta_2);
        double theta_3 =
            acos((r_f - r_g - l_gp * cos(theta_2 - theta_0)) / l_pf) - xi_0 + psi_0;
        phi_3 = actuator3.calc_phi(theta_3);
    }

    void calc_trajectory(double coords[][3], double angles[][3], int list_length) {
        for(int i = 0; i<list_length;i++) {
          backward_transform(coords[i][0], coords[i][1], coords[i][2], angles[i][0], angles[i][1], angles[i][2]);
        }
    }

    void calc_PWM(double angles[][3], int pwm[][3], int list_length) {
      for(int i = 0; i<list_length;i++) {
        pwm[i][0]=actuator1.calc_PWM(angles[i][0]);
        pwm[i][1]=actuator2.calc_PWM(angles[i][1]);
        pwm[i][2]=actuator3.calc_PWM(angles[i][2]);
      }
    }

    void update_actuator_angles(double phi_1, double phi_2, double phi_3) {
        actuator1.set_cur_phi(phi_1);
        actuator2.set_cur_phi(phi_2);
        actuator3.set_cur_phi(phi_3);
        actuator1.set_cur_theta(actuator1.calc_theta(phi_1));
        actuator2.set_cur_theta(actuator2.calc_theta(phi_2));
        actuator3.set_cur_theta(actuator3.calc_theta(phi_3));
    }

    void update_cur_pos(double phi_1, double phi_2, double phi_3) {
        forward_transform(phi_1, phi_2, phi_3, cur_x_f, cur_y_f, cur_z_f);
        update_actuator_angles(phi_1, phi_2, phi_3);
    }

    void update_cur_phi(double x_f, double y_f, double z_f) {
        cur_x_f = x_f;
        cur_y_f = y_f;
        cur_z_f = z_f;
        double phi_1, phi_2, phi_3;
        backward_transform(x_f, y_f, z_f, phi_1, phi_2, phi_3);
        update_actuator_angles(phi_1, phi_2, phi_3);
    }
};