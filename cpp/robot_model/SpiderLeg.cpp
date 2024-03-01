#include "math.h"
#include "FourBarLinkage.h"
#include "SpiderLeg.h"

const float x_j = 43.5, y_j = 42, r_g = 66.5, z_g = -24, l_gp = 30.5, l_pf = 78.2;
const double psi_0 = 131.5 * M_PI / 180.0;
const double xi_0 = 147.5 * M_PI / 180.0;
const double  theta_0 = (90 - 9.5) * M_PI / 180.0;
const double  theta_leg = 33.7 * M_PI / 180.0;
const float act1_l_sg = 42.5, act1_l_sa = 14.5, act1_l_ab = 38, act1_l_gb = 27.8, act1_phi_0= 90;
const float act2_l_sg = 35.6, act2_l_sa = 14.5, act2_l_ab = 35.6, act2_l_gb = 25.6, act2_phi_0= 99.5;
const float act3_l_sg = 35.6, act3_l_sa = 14.5, act3_l_ab = 26, act3_l_gb = 38.5, act3_phi_0= 99.5;

SpiderLeg::SpiderLeg(int dir_x = 1, int dir_y = 1, std::string name = ""): 
  actuator1(act1_l_sg, act1_l_sa, act1_l_ab, act1_l_gb,  act1_phi_0),
  actuator2(act2_l_sg, act2_l_sa, act2_l_ab, act2_l_gb, act2_phi_0),
  actuator3(act3_l_sg, act3_l_sa, act3_l_ab, act3_l_gb, act3_phi_0) {
  this->dir_x = dir_x;
  this->dir_y = dir_y;
  this->name = name;
  forward_transform(this->actuator1.get_phi_0(), this->actuator2.get_phi_0(), this->actuator3.get_phi_0(), &this->init_x_f, &this->init_y_f, &this->init_z_f);
  this->init_phi = atan2(init_x_f, init_y_f);
  this->cur_x_f, this->cur_y_f, this->cur_z_f = init_x_f, init_y_f, init_z_f;
}

void SpiderLeg::forward_transform(double phi_1, double phi_2, double phi_3, double* x_f, double* y_f, double* z_f) {
  double theta_1 = this->actuator1.calc_theta(phi_1);
  double theta_2 = this->actuator2.calc_theta(phi_2);
  double theta_3 = this->actuator3.calc_theta(phi_3);
  double r_f = r_g + l_gp * cos(theta_2 - theta_0) + l_pf * cos(xi_0 - psi_0 + theta_3);
  *z_f = z_g - l_gp * sin(theta_2 - theta_0) + l_pf * sin(xi_0 - psi_0 + theta_3);
  *x_f = this->dir_x * (x_j - r_f * cos(theta_1 + theta_leg));
  *y_f = this->dir_y * (y_j + r_f * sin(theta_1 + theta_leg));
}

void SpiderLeg::backward_transform(double x_f, double y_f, double z_f, double* phi_1, double* phi_2, double* phi_3) {
  double r_f = sqrt((x_f - this->dir_x * x_j) * (x_f - this->dir_x * x_j) + (y_f - this->dir_y * y_j) * (y_f - this->dir_y * y_j));
  double theta_1 = acos((x_j - x_f * this->dir_x) / r_f) - theta_leg;
  *phi_1 = this->actuator1.calc_phi(theta_1);
  double vec_length = (r_f - r_g) * (r_f - r_g) + (z_f - z_g) * (z_f - z_g);
  double theta_2 =
      acos((l_gp * l_gp - l_pf * l_pf + vec_length) / (2 * l_gp * sqrt(vec_length))) -
      atan2(z_f - z_g, r_f - r_g) + theta_0;
  *phi_2 = this->actuator2.calc_phi(theta_2);
  double theta_3 =
      acos((r_f - r_g - l_gp * cos(theta_2 - theta_0)) / l_pf) - xi_0 + psi_0;
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