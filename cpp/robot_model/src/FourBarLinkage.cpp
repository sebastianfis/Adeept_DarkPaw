#include "math.h"
#include "Arduino.h"
#include "FourBarLinkage.h"


/**
 * @brief Construct a FourBarLinkage object
 *
 * Initializes a planar four-bar linkage defined by its link lengths
 * and an initial actuator angle.
 *
 *
 *
 * Geometry:
 *         A ●────────────● B
 *         ╲     l_ab      ╱
 *          ╲             ╱
 *       l_sa╲           ╱ l_gb
 *            ╲  φ      ╱
 *             ●───────●
 *             S  l_sg  G
 *  - s : servo pivot
 *  - g : ground pivot
 *  - a : joint A
 *  - b : joint B
 *
 * @param l_sg Length between servo pivot (s) and ground pivot (g)
 * @param l_sa Length between servo pivot (s) and joint A
 * @param l_ab Length between joint A and joint B
 * @param l_gb Length between ground pivot (g) and joint B
 * @param phi_0 Initial actuator angle in degrees
 */

FourBarLinkage::FourBarLinkage(float l_sg, float l_sa, float l_ab, float l_gb, float phi_0) {
    this->l_sg = l_sg;
    this->l_sa = l_sa;
    this->l_ab = l_ab;
    this->l_gb = l_gb;

    // Default motion limits (radians)
    this->phi_max = M_PI;
    this->phi_min = -M_PI;
    this->theta_max = M_PI;
    this->theta_min = -M_PI;

    // Convert initial angle from degrees to radians
    this->cur_phi = phi_0 * M_PI / 180.0;
    this->cur_theta = calc_theta(cur_phi);

    this->phi_0 = phi_0 * M_PI / 180.0;

    // Default servo calibration
    this->pwm_0 = 330;
    this->actuator_direction = 1;
}

/**
 * @brief Set PWM calibration parameters
 *
 * @param pwm_value PWM value corresponding to the reference angle phi_0
 * @param actuator_dir Direction of actuator motion (+1 or -1)
 */
void FourBarLinkage::set_pwm_init(short pwm_value, short actuator_dir) {
    assert(actuator_dir == 1 || actuator_dir == -1 && "actuator_direction must be 1 or -1!");
    this->pwm_0 = pwm_value;
    this->actuator_direction = actuator_dir;
}

/**
 * @brief Convert actuator angle to PWM signal
 *
 * Maps the angular displacement from phi_0 to a PWM value suitable
 * for a servo motor. Output is clamped to valid servo limits.
 *
 * @param phi Actuator angle in radians
 * @return PWM signal (typically 100–560)
 */
short FourBarLinkage::calc_PWM(float phi) {
    float pwm_value = this->pwm_0 + this->actuator_direction * (phi - this->phi_0) * 460 / M_PI;
    
    // Clamp to safe servo range
    if (pwm_value < 100) {
        pwm_value = 100;
    } else if (pwm_value > 560) {
        pwm_value = 560;
    }
    return short(pwm_value);
}

/**
 * @brief Compute output angle theta from input angle phi
 *
 * Uses planar triangle geometry (law of cosines) to compute
 * the resulting linkage angle.
 *
 * @param phi Input actuator angle (radians)
 * @return Output angle theta (radians)
 */
float FourBarLinkage::calc_theta(float phi) {
    // Distance between joints A and G
    float l_ag = sqrt(this->l_sa * this->l_sa + this->l_sg * this->l_sg - 2 * this->l_sa * this->l_sg * cos(phi));
    // assert((this->phi_min <= phi && phi <= this->phi_max) &&
    // "Movement not possible, because l_ag > l_ab + l_gb");

    // Clamp phi to valid range
    if (phi < this->phi_min){
        phi = this->phi_min;
    } 
    if (phi > this->phi_max){
        phi = this->phi_max;
    }

    // First internal angle
    float v1 = constrain((this->l_sg * this->l_sg + l_ag * l_ag - this->l_sa * this->l_sa) /
                                    (2 * this->l_sg * l_ag), -1.0, 1.0);
    float theta_1 = acos(v1);

    // Second internal angle
    float v2 = constrain((this->l_gb * this->l_gb + l_ag * l_ag - this->l_ab * this->l_ab) /
                                    (2 * this->l_gb * l_ag), -1.0, 1.0);
    float theta_2 = acos(v2);
    float theta = theta_1 + theta_2;
    return theta;
}

/**
 * @brief Compute actuator angle phi from output angle theta
 *
 * Inverse kinematics of the four-bar linkage.
 *
 * @param theta Output angle (radians)
 * @return Required actuator angle phi (radians)
 */
float FourBarLinkage::calc_phi(float theta) {
    // Distance between joints B and S
    float l_bs = sqrt(this->l_gb * this->l_gb + this->l_sg * this->l_sg - 2 * this->l_gb *this-> l_sg * cos(theta));
    // assert((this->theta_min <= theta && theta <= this->theta_max) &&
    //         "Movement not possible, because l_bs > l_ab + l_sa");

    // Clamp theta to valid range
    if (theta < this->theta_min){
        theta = this->theta_min;
    } 
    if (theta > this->theta_max){
        theta = this->theta_max;
    }
    float v1 =  constrain((this->l_sg * this->l_sg + l_bs * l_bs - this->l_gb * this->l_gb) /
                                  (2 * this->l_sg * l_bs), -1.0, 1.0);
    float phi_1 = acos(v1);
    float v2 =  constrain((this->l_sa * this->l_sa + l_bs * l_bs - this->l_ab * this->l_ab) /
                                  (2 * l_sa * l_bs), -1.0, 1.0);
    double phi_2 = acos(v2);
    double phi = phi_1 + phi_2;
    return phi;
}

/**
 * @brief Calculate valid motion limits for the linkage
 *
 * Computes physical limits for phi and theta based on
 * linkage geometry and an initial reference angle.
 *
 * @param phi_0 Reference actuator angle (radians)
 */
void FourBarLinkage::calc_limits(float phi_0) {
    // Maximum AG distance
    float l_ag_max = this->l_ab + this->l_gb;
    float v1 = constrain((this->l_sa * this->l_sa + this->l_sg * this->l_sg - l_ag_max * l_ag_max) /
                                     (2 * this->l_sa * this->l_sg), -1.0, 1.0);
    float phi_max = acos(v1);

    // Limit actuator range to ±90° around reference
    if (this->phi_max > phi_0 + M_PI / 2) {
        this->phi_max = phi_0 + M_PI / 2;
    } else {
        this->phi_max = phi_max;
    }

    this->theta_min = calc_theta(phi_max);

    // Maximum BS distance
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

/**
 * @brief Get current actuator angle
 * @return Current phi (radians)
 */
float FourBarLinkage::get_cur_phi() {
  return this->cur_phi;
}

/**
 * @brief Set current actuator angle
 * @param phi Angle in radians
 */
void FourBarLinkage::set_cur_phi(float phi) {
  this->cur_phi = phi;
}

/**
 * @brief Set current output angle
 * @param theta Angle in radians
 */
void FourBarLinkage::set_cur_theta(float theta) {
  this->cur_theta = theta;
}

/**
 * @brief Get reference actuator angle
 * @return Reference angle phi_0 (radians)
 */
float FourBarLinkage::get_phi_0() {
    return this->phi_0;
}
