#ifndef FourBarLinkage_h
#define FourBarLinkage_h

#include "math.h"
#include "Arduino.h"

/**
 * @class FourBarLinkage
 * @brief Planar four-bar linkage kinematics with servo PWM mapping
 *
 * This class models a closed-loop planar four-bar linkage consisting of:
 *
 *        A ●────────────● B
 *         ╲     l_ab      ╱
 *          ╲             ╱
 *       l_sa╲           ╱ l_gb
 *            ╲         ╱
 *             ●───────●
 *             S  l_sg  G
 *
 * The linkage forms a single kinematic loop:
 *
 *   S → A → B → G → S
 *
 * - S : Servo pivot (input crank)
 * - G : Ground pivot (fixed)
 * - A : Joint A
 * - B : Joint B
 *
 * Angles:
 * - φ (phi)   : Input angle at S between links SA and SG
 * - θ (theta) : Output angle at G between links GB and GS
 *
 * The class supports:
 * - Forward kinematics (phi → theta)
 * - Inverse kinematics (theta → phi)
 * - Physical motion limit calculation
 * - Mapping angles to servo PWM values
 */
class FourBarLinkage {

private:
    /**
     * @brief Link lengths (geometry parameters)
     */
    float l_sg;  ///< Length of ground link between S and G
    float l_sa;  ///< Length of input crank between S and A
    float l_ab;  ///< Length of coupler link between A and B
    float l_gb;  ///< Length of output rocker between G and B

    /**
     * @brief Motion limits (radians)
     *
     * These limits are computed based on linkage geometry
     * and ensure physically valid configurations.
     */
    float phi_max;    ///< Maximum allowed actuator angle
    float phi_min;    ///< Minimum allowed actuator angle
    float theta_max;  ///< Maximum allowed output angle
    float theta_min;  ///< Minimum allowed output angle

    /**
     * @brief Current and reference state variables
     */
    float cur_phi;   ///< Current actuator angle (radians)
    float cur_theta; ///< Current output angle (radians)
    float phi_0;     ///< Reference actuator angle (radians)

    /**
     * @brief Servo configuration
     */
    short actuator_direction; ///< +1 or -1 depending on servo orientation
    short pwm_0;              ///< PWM value corresponding to phi_0

public:
    /**
     * @brief Construct a FourBarLinkage object
     *
     * @param l_sg Length of ground link SG
     * @param l_sa Length of input crank SA
     * @param l_ab Length of coupler link AB
     * @param l_gb Length of output rocker GB
     * @param phi_0 Initial actuator angle (degrees)
     */
    FourBarLinkage(float l_sg, float l_sa, float l_ab, float l_gb, float phi_0);

    /**
     * @brief Initialize servo PWM calibration
     *
     * @param pwm_value PWM value corresponding to reference angle phi_0
     * @param actuator_direction Direction of actuator (+1 or -1)
     */
    void set_pwm_init(short pwm_value, short actuator_direction);

    /**
     * @brief Convert actuator angle to PWM signal
     *
     * Maps the difference between a given angle and the reference angle
     * to a servo PWM value. Output is clamped to safe servo limits.
     *
     * @param phi Actuator angle (radians)
     * @return PWM value
     */
    short calc_PWM(float phi);

    /**
     * @brief Forward kinematics: compute theta from phi
     *
     * Uses planar triangle geometry and the law of cosines.
     *
     * @param phi Actuator angle (radians)
     * @return Output angle theta (radians)
     */
    float calc_theta(float phi);

    /**
     * @brief Inverse kinematics: compute phi from theta
     *
     * Solves the linkage geometry for the required actuator angle.
     *
     * @param theta Output angle (radians)
     * @return Actuator angle phi (radians)
     */
    float calc_phi(float theta);

    /**
     * @brief Compute physical motion limits
     *
     * Determines valid ranges for phi and theta based on
     * linkage geometry and a reference angle.
     *
     * @param phi_0 Reference actuator angle (radians)
     */
    void calc_limits(float phi_0);

    /**
     * @brief Get the current actuator angle
     * @return Current phi (radians)
     */
    float get_cur_phi();

    /**
     * @brief Set the current actuator angle
     * @param phi Angle in radians
     */
    void set_cur_phi(float phi);

    /**
     * @brief Set the current output angle
     * @param theta Angle in radians
     */
    void set_cur_theta(float theta);

    /**
     * @brief Get the reference actuator angle
     * @return Reference phi_0 (radians)
     */
    float get_phi_0();
};

#endif
