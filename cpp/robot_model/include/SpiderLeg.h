#ifndef SpiderLeg_h
#define SpiderLeg_h

#include "math.h"
#include "Arduino.h"
#include "FourBarLinkage.h"

/**
 * @class SpiderLeg
 * @brief 3-DOF spider leg kinematics using three four-bar actuators
 *
 * This class models a single robotic spider leg composed of three
 * four-bar linkages:
 *
 *  - Actuator 1: Hip yaw (rotation in the XY plane)
 *  - Actuator 2: Hip pitch (vertical lift)
 *  - Actuator 3: Knee extension
 *
 * The leg supports both forward and inverse kinematics, trajectory
 * generation, and conversion of joint angles to servo PWM values.
 *
 * Coordinate frame:
 *  - Origin at robot body center
 *  - X/Y define the horizontal plane
 *  - Z points downward
 *
 * Direction flags (dir_x, dir_y) allow mirroring for left/right legs.
 */
class SpiderLeg {

private:
    /**
     * @brief Human-readable leg identifier
     */
    const char* name;

    /**
     * @brief Direction multipliers for coordinate mirroring
     *
     * Used to mirror kinematics between left/right or front/back legs.
     * Valid values are +1 or −1.
     */
    short dir_x; ///< X-axis direction multiplier
    short dir_y; ///< Y-axis direction multiplier

    /**
     * @brief Initial foot position and polar representation
     */
    float init_x_f;  ///< Initial foot X position
    float init_y_f;  ///< Initial foot Y position
    float init_z_f;  ///< Initial foot Z position
    float init_phi;  ///< Initial foot polar angle
    float init_r;    ///< Initial foot radial distance

    /**
     * @brief Current foot position
     */
    float cur_x_f; ///< Current foot X position
    float cur_y_f; ///< Current foot Y position
    float cur_z_f; ///< Current foot Z position

public:
    /**
     * @brief Four-bar actuators controlling the leg
     *
     * actuator1 → Hip yaw
     * actuator2 → Hip pitch
     * actuator3 → Knee
     */
    FourBarLinkage actuator1;
    FourBarLinkage actuator2;
    FourBarLinkage actuator3;

    /**
     * @brief Construct a SpiderLeg
     *
     * Initializes all three four-bar actuators and computes
     * the initial foot position using forward kinematics.
     *
     * @param dir_x Direction multiplier for X (+1 or −1)
     * @param dir_y Direction multiplier for Y (+1 or −1)
     * @param name  Human-readable leg name
     */
    SpiderLeg(short dir_x, short dir_y, const char* name);

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
    void forward_transform(float phi_1, float phi_2, float phi_3, float* x_f, float* y_f, float* z_f);

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
    void backward_transform(float x_f, float y_f, float z_f, float* phi_1, float* phi_2, float* phi_3);

    /**
     * @brief Compute actuator angles for a trajectory
     *
     * @param coords Input foot coordinates [N][3]
     * @param angles Output actuator angles [N][3]
     * @param list_length Number of trajectory points
     */
    void calc_trajectory(float coords[][3], float angles[][3], int list_length);

    /**
     * @brief Convert actuator angles to PWM values
     *
     * @param angles Input angles [N][3]
     * @param pwm    Output PWM values [N][3]
     * @param list_length Number of samples
     */
    void calc_PWM(float angles[][3], short pwm[][3], short list_length);

    /**
     * @brief Update internal actuator angle states
     */
    void update_actuator_angles(float phi_1, float phi_2, float phi_3);

    /**
     * @brief Update current foot position from actuator angles
     */
    void update_cur_pos(float phi_1, float phi_2, float phi_3);

    /**
     * @brief Update actuator angles from desired foot position
     */
    void update_cur_phi(float x_f, float y_f, float z_f);

    /**
     * @brief Get current actuator angles
     *
     * @param angles Output array [3]
     */
    void get_actuator_angles(float angles[3]);

    /**
     * @brief Get current foot position
     *
     * @param coord Output array [3]
     */
    void get_cur_pos(float coord[3]);

    /**
     * @brief Get initial foot position
     *
     * @param coord Output array [3]
     */
    void get_init_pos(float coord[3]);

    /**
     * @brief Get initial polar angle of the foot
     *
     * @return Angle in radians
     */
    float get_init_phi();

    /**
     * @brief Get initial radial distance of the foot
     *
     * @return Radius
     */
    float get_init_r();

    /**
     * @brief Get leg name
     */
    const char* get_name();
};

#endif
