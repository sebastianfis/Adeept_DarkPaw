#include "math.h"
#include "FourBarLinkage.h"
#include "SpiderLeg.h"

SpiderLeg::SpiderLeg(double x_j = 43.5, double y_j = 42, int dir_x = 1, int dir_y = 1, double r_g = 66.5, 
                     double z_g = -23, double l_gp = 30.5, double l_pf = 78.2, double psi_0 = 131.5, double xi_0 = 147.5, 
                     double theta_0 = 90 - 9.5, double theta_leg = 33.7, std::string name = ""): 
  actuator1(42.5, 14.5, 38, 27.8, 90),
  actuator2(35.6, 14.5, 35.6, 25.6, 99.5),
  actuator3(35.6, 14.5, 26, 38.5, 99.5) {
  this->x_j = x_j;
  this->y_j = y_j;
  this->dir_x = dir_x;
  this->dir_y = dir_y;
  this->name = name;
  this->r_g = r_g;
  this->z_g = z_g;
  this->l_gp = l_gp;
  this->l_pf = l_pf;
  this->psi_0 = psi_0 * M_PI / 180.0;
  this->xi_0 = xi_0 * M_PI / 180.0;
  this->theta_0 = theta_0 * M_PI / 180.0;
  this->theta_leg = theta_leg * M_PI / 180.0;
  forward_transform(this->actuator1.get_phi_0(), this->actuator2.get_phi_0(), this->actuator3.get_phi_0(), &this->init_x_f, &this->init_y_f, &this->init_z_f);
  this->init_phi = atan2(init_x_f, init_y_f);
  this->cur_x_f, this->cur_y_f, this->cur_z_f = init_x_f, init_y_f, init_z_f;
}

void SpiderLeg::forward_transform(double phi_1, double phi_2, double phi_3, double* x_f, double* y_f, double* z_f) {
  double theta_1 = this->actuator1.calc_theta(phi_1);
  double theta_2 = this->actuator2.calc_theta(phi_2);
  double theta_3 = this->actuator3.calc_theta(phi_3);
  double r_f = this->r_g + this->l_gp * cos(theta_2 - this->theta_0) + this->l_pf * cos(this->xi_0 - this->psi_0 + theta_3);
  *z_f = this->z_g - this->l_gp * sin(theta_2 - this->theta_0) + this->l_pf * sin(this->xi_0 - this->psi_0 + theta_3);
  *x_f = this->x_j - this->dir_x * r_f * cos(theta_1 + this->theta_leg);
  *y_f = this->y_j + this->dir_y * r_f * sin(theta_1 + this->theta_leg);
}

void SpiderLeg::backward_transform(double x_f, double y_f, double z_f, double* phi_1, double* phi_2, double* phi_3) {
  double r_f = sqrt((x_f - this->x_j) * (x_f - this->x_j) + (y_f - this->y_j) * (y_f - this->y_j));
  double theta_1 = acos((this->x_j - x_f) / this->dir_x / r_f) - this->theta_leg;
  *phi_1 = this->actuator1.calc_phi(theta_1);
  double vec_length = (r_f - this->r_g) * (r_f - this->r_g) + (z_f - this->z_g) * (z_f - this->z_g);
  double theta_2 =
      acos((this->l_gp * this->l_gp - this->l_pf * this->l_pf + vec_length) / (2 * this->l_gp * sqrt(vec_length))) -
      atan2(z_f - this->z_g, r_f - this->r_g) + this->theta_0;
  *phi_2 = this->actuator2.calc_phi(theta_2);
  double theta_3 =
      acos((r_f - this->r_g - this->l_gp * cos(theta_2 - this->theta_0)) / this->l_pf) - this->xi_0 + this->psi_0;
  *phi_3 = this->actuator3.calc_phi(theta_3);
}

void SpiderLeg::calc_trajectory(double coords[][3], double angles[][3], int list_length) {
    for(int i = 0; i<list_length;i++) {
      backward_transform(coords[i][0], coords[i][1], coords[i][2], &angles[i][0], &angles[i][1], &angles[i][2]);
    }
}

void SpiderLeg::calc_PWM(double angles[][3], int pwm[][3], int list_length) {
  for(int i = 0; i<list_length;i++) {
    pwm[i][0]=this->actuator1.calc_PWM(angles[i][0]);
    pwm[i][1]=this->actuator2.calc_PWM(angles[i][1]);
    pwm[i][2]=this->actuator3.calc_PWM(angles[i][2]);
  }
}

void SpiderLeg::update_actuator_angles(double phi_1, double phi_2, double phi_3) {
    this->actuator1.set_cur_phi(phi_1);
    this->actuator2.set_cur_phi(phi_2);
    this->actuator3.set_cur_phi(phi_3);
    this->actuator1.set_cur_theta(this->actuator1.calc_theta(phi_1));
    this->actuator2.set_cur_theta(this->actuator2.calc_theta(phi_2));
    this->actuator3.set_cur_theta(this->actuator3.calc_theta(phi_3));
}

void SpiderLeg::update_cur_pos(double phi_1, double phi_2, double phi_3) {
    forward_transform(phi_1, phi_2, phi_3, &this->cur_x_f, &this->cur_y_f, &this->cur_z_f);
    update_actuator_angles(phi_1, phi_2, phi_3);
}

void SpiderLeg::update_cur_phi(double x_f, double y_f, double z_f) {
    this->cur_x_f = x_f;
    this->cur_y_f = y_f;
    this->cur_z_f = z_f;
    double phi_1, phi_2, phi_3;
    backward_transform(x_f, y_f, z_f, &phi_1, &phi_2, &phi_3);
    update_actuator_angles(phi_1, phi_2, phi_3);
}

void SpiderLeg::get_actuator_angles(double angles[3]) {
  angles[0]=this->actuator1.get_cur_phi();
  angles[1]=this->actuator2.get_cur_phi();
  angles[2]=this->actuator3.get_cur_phi();
}

std::string SpiderLeg::get_name() {
  return this->name;
}