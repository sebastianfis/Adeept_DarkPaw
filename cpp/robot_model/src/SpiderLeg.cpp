#include "math.h"
#include "FourBarLinkage.h"
#include "SpiderLeg.h"

/**
 * @file SpiderLeg.cpp
 * @brief Implementation of a 3-DOF spider leg using three four-bar actuators
 *
 * This file implements forward and inverse kinematics for a single spider leg.
 * The leg consists of three actuated four-bar linkages controlling:
 *  - Hip yaw (rotation in XY plane)
 *  - Hip pitch (vertical motion)
 *  - Knee extension
 *
 * All dimensions are in millimeters.
 * All angles are in radians unless stated otherwise.
 */

/* ===== Fixed geometry constants ===== */

// Joint offset in body frame
const float x_j = 43.5;   ///< X-offset of leg joint from body center
const float y_j = 42.0;   ///< Y-offset of leg joint from body center

// Ground reference point
const float r_g = 66.5;   ///< Ground radial offset
const float z_g = -24.0;  ///< Ground height

// Link lengths
const float l_gp = 30.5;  ///< Ground to proximal joint length
const float l_pf = 78.2;  ///< Proximal to foot length

/* ===== Fixed angle offsets ===== */
const float psi_0 = 131.5 * M_PI / 180.0;   ///< Knee reference angle
const float xi_0 = 147.5 * M_PI / 180.0;   ///< Knee linkage offset
const float theta_0 = (90 - 9.5) * M_PI / 180.0; ///< Hip pitch offset
const float theta_leg = 33.7 * M_PI / 180.0;  ///< Leg mounting angle

/* ===== Four-bar actuator parameters ===== */

// Actuator 1: Hip yaw
const float act1_l_sg = 42.5;
const float act1_l_sa = 14.5;
const float act1_l_ab = 38.0;
const float act1_l_gb = 27.8;
const float act1_phi_0 = 90.0;

// Actuator 2: Hip pitch
const float act2_l_sg = 35.6;
const float act2_l_sa = 14.5;
const float act2_l_ab = 35.6;
const float act2_l_gb = 25.6;
const float act2_phi_0 = 99.5;

// Actuator 3: Knee
const float act3_l_sg = 35.6;
const float act3_l_sa = 14.5;
const float act3_l_ab = 26.0;
const float act3_l_gb = 38.5;
const float act3_phi_0 = 99.5;

/**
 * @brief Construct a SpiderLeg object
 *
 * Initializes three four-bar actuators and computes the initial
 * foot position using forward kinematics.
 *
 * @param dir_x Direction multiplier for X (+1 or -1)
 * @param dir_y Direction multiplier for Y (+1 or -1)
 * @param name  Human-readable leg name
 */
SpiderLeg::SpiderLeg(short dir_x = 1, short dir_y = 1, const char* name = "  "):
  actuator1(act1_l_sg, act1_l_sa, act1_l_ab, act1_l_gb,  act1_phi_0),
  actuator2(act2_l_sg, act2_l_sa, act2_l_ab, act2_l_gb, act2_phi_0),
  actuator3(act3_l_sg, act3_l_sa, act3_l_ab, act3_l_gb, act3_phi_0) {
  this->dir_x = dir_x;
  this->dir_y = dir_y;
  this->name = name;
  // Compute initial foot position
  forward_transform(this->actuator1.get_phi_0(), this->actuator2.get_phi_0(), this->actuator3.get_phi_0(), &this->init_x_f, &this->init_y_f, &this->init_z_f);
  // Polar representation of initial foot position
  this->init_phi = atan2(init_x_f, init_y_f);
  this->init_r = sqrt(init_x_f* init_x_f + init_y_f *init_y_f);
  // Initialize current position
  this->cur_x_f = init_x_f;
  this->cur_y_f = init_y_f;
  this->cur_z_f = init_z_f;
}

/**
 * @brief Forward kinematics
 *
 * Computes foot position from actuator angles.
 *
 * @param phi_1 Actuator 1 angle (rad)
 * @param phi_2 Actuator 2 angle (rad)
 * @param phi_3 Actuator 3 angle (rad)
 * @param x_f   Output foot X position
 * @param y_f   Output foot Y position
 * @param z_f   Output foot Z position
 */
void SpiderLeg::forward_transform(float phi_1, float phi_2, float phi_3, float* x_f, float* y_f, float* z_f) {
  float theta_1 = this->actuator1.calc_theta(phi_1);
  float theta_2 = this->actuator2.calc_theta(phi_2);
  float theta_3 = this->actuator3.calc_theta(phi_3);
  // Radial distance of foot
  float r_f = r_g + l_gp * cos(theta_2 - theta_0) + l_pf * cos(xi_0 - psi_0 + theta_3);
  // Vertical position
  *z_f = z_g - l_gp * sin(theta_2 - theta_0) + l_pf * sin(xi_0 - psi_0 + theta_3);
  // Cartesian position
  *x_f = this->dir_x * (x_j - r_f * cos(theta_1 + theta_leg));
  *y_f = this->dir_y * (y_j + r_f * sin(theta_1 + theta_leg));
}

/**
 * @brief Inverse kinematics
 *
 * Computes actuator angles required to reach a given foot position.
 *
 * @param x_f   Desired foot X position
 * @param y_f   Desired foot Y position
 * @param z_f   Desired foot Z position
 * @param phi_1 Output actuator 1 angle
 * @param phi_2 Output actuator 2 angle
 * @param phi_3 Output actuator 3 angle
 */
void SpiderLeg::backward_transform(float x_f, float y_f, float z_f, float* phi_1, float* phi_2, float* phi_3) {
  float r_f = sqrt((x_f - this->dir_x * x_j) * (x_f - this->dir_x * x_j) + (y_f - this->dir_y * y_j) * (y_f - this->dir_y * y_j));
  float theta_1 = acos((x_j - x_f * this->dir_x) / r_f) - theta_leg;
  *phi_1 = this->actuator1.calc_phi(theta_1);
  float vec_length = (r_f - r_g) * (r_f - r_g) + (z_f - z_g) * (z_f - z_g);
  float theta_2 =
      acos((l_gp * l_gp - l_pf * l_pf + vec_length) / (2 * l_gp * sqrt(vec_length))) -
      atan2(z_f - z_g, r_f - r_g) + theta_0;
  *phi_2 = this->actuator2.calc_phi(theta_2);
  float theta_3 =
      acos((r_f - r_g - l_gp * cos(theta_2 - theta_0)) / l_pf) - xi_0 + psi_0;
  *phi_3 = this->actuator3.calc_phi(theta_3);
}

/**
 * @brief Compute actuator angles for a trajectory
 *
 * @param coords Input foot coordinates [N][3]
 * @param angles Output actuator angles [N][3]
 * @param list_length Number of trajectory points
 */
void SpiderLeg::calc_trajectory(float coords[][3], float angles[][3], int list_length) {
    for(int i = 0; i<list_length;i++) {
      backward_transform(coords[i][0], coords[i][1], coords[i][2], &angles[i][0], &angles[i][1], &angles[i][2]);
    }
}

/**
 * @brief Convert actuator angles to PWM values
 *
 * @param angles Input angles [N][3]
 * @param pwm    Output PWM values [N][3]
 * @param list_length Number of samples
 */
void SpiderLeg::calc_PWM(float angles[][3], short pwm[][3], short list_length) {
  for(short i = 0; i < list_length; i++) {
    pwm[i][0] = this->actuator1.calc_PWM(angles[i][0]);
    pwm[i][1] = this->actuator2.calc_PWM(angles[i][1]);
    pwm[i][2] = this->actuator3.calc_PWM(angles[i][2]);
  }
}

/**
 * @brief Update actuator internal state
 */
void SpiderLeg::update_actuator_angles(float phi_1, float phi_2, float phi_3) {
    this->actuator1.set_cur_phi(phi_1);
    this->actuator2.set_cur_phi(phi_2);
    this->actuator3.set_cur_phi(phi_3);
    this->actuator1.set_cur_theta(this->actuator1.calc_theta(phi_1));
    this->actuator2.set_cur_theta(this->actuator2.calc_theta(phi_2));
    this->actuator3.set_cur_theta(this->actuator3.calc_theta(phi_3));
}

/**
 * @brief Update current foot position from actuator angles
 */
void SpiderLeg::update_cur_pos(float phi_1, float phi_2, float phi_3) {
    forward_transform(phi_1, phi_2, phi_3, &this->cur_x_f, &this->cur_y_f, &this->cur_z_f);
    update_actuator_angles(phi_1, phi_2, phi_3);
}

/**
 * @brief Update actuator angles from desired foot position
 */
void SpiderLeg::update_cur_phi(float x_f, float y_f, float z_f) {
    this->cur_x_f = x_f;
    this->cur_y_f = y_f;
    this->cur_z_f = z_f;
    float phi_1, phi_2, phi_3;
    backward_transform(x_f, y_f, z_f, &phi_1, &phi_2, &phi_3);
    update_actuator_angles(phi_1, phi_2, phi_3);
}

/**
 * @brief Get current actuator angles
 */
void SpiderLeg::get_actuator_angles(float angles[3]) {
  angles[0]=this->actuator1.get_cur_phi();
  angles[1]=this->actuator2.get_cur_phi();
  angles[2]=this->actuator3.get_cur_phi();
}

/**
 * @brief Get leg name
 */
const char* SpiderLeg::get_name() {
  return this->name;
}

/**
 * @brief Get current foot position
 */
void SpiderLeg::get_cur_pos(float coord[3]) {
  coord[0] = this->cur_x_f;
  coord[1] = this->cur_y_f;
  coord[2] = this->cur_z_f;
}

/**
 * @brief Get initial foot position
 */
void SpiderLeg::get_init_pos(float coord[3]) {
  coord[0] = this->init_x_f;
  coord[1] = this->init_y_f;
  coord[2] = this->init_z_f;
}

/**
 * @brief Get initial polar angle of foot
 */
float SpiderLeg::get_init_phi() {
  return this->init_phi;
}

/**
 * @brief Get initial radial distance of foot
 */
float SpiderLeg::get_init_r() {
  return this->init_r;
}
