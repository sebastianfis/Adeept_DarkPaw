#ifndef Gait_h
#define Gait_h
#include "SpiderLeg.h"

/**
 * @brief Generate a stepping (swing phase) trajectory.
 *
 * Creates a smooth foot trajectory between two Cartesian points:
 * - X and Y move linearly
 * - Z follows an arched profile to lift the foot off the ground
 *
 * @param starting_point 3D start position of the foot [x, y, z]
 * @param end_point      3D end position of the foot [x, y, z]
 * @param step_height    Maximum foot lift height during swing
 * @param result         Output array of size [n][3] containing the trajectory
 * @param n              Number of trajectory samples
 */
void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n);
/**
 * @brief Generate a straight-line trajectory.
 *
 * Used for the stance phase where the foot remains in contact
 * with the ground while the body moves.
 *
 * @param starting_point 3D start position of the foot
 * @param end_point      3D end position of the foot
 * @param result         Output array of size [n][3]
 * @param n              Number of trajectory samples
 */
void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n);
/**
 * @brief Generate a partial circular trajectory in the XY plane.
 *
 * Used for turning gaits where the robot rotates around
 * its vertical axis.
 *
 * @param starting_point 3D start position of the foot
 * @param end_point      3D end position of the foot
 * @param result         Output array of size [n][3]
 * @param n              Number of trajectory samples
 */
void generate_partial_circle(float starting_point[3], float end_point[3], float result[][3], short n);

/**
 * @class Gait
 * @brief High-level gait generator for a quadruped robot.
 *
 * This class:
 * - Generates Cartesian foot trajectories for all four legs
 * - Converts trajectories into actuator angles and PWM signals
 * - Supports translation (x/y) and rotation (t) gaits
 * - Handles gait initialization and cyclic walking
 *
 * The gait is represented as precomputed lookup tables:
 * - Cartesian foot positions
 * - Corresponding servo PWM values
 *
 * These tables can be replayed in real-time by a controller.
 */
class Gait {

  private:

    /**
     * @brief Direction inversion flag
     *
     * If true, gait direction is reversed.
     */
    bool inv_direction;

    /**
     * @brief Controller update frequency (Hz)
     */
    float freq;

    /**
     * @brief Gait direction
     *
     * 'x' → forward/backward translation
     * 'y' → lateral translation
     * 't' → rotation (turning)
     */
    char direction;

    /**
     * @brief Step length
     *
     * Linear distance for x/y gaits or angular distance for turning gaits.
     */
    float step_length;

    /**
     * @brief Maximum foot lift height during swing phase
     */
    float step_height;

    /**
     * @brief Desired gait velocity
     */
    float velocity;

    /**
     * @brief Average leg radius (used for turning gaits)
     */
    float r;

       /**
     * @brief Represeantaion of name
     */
    const char* name;

    /**
     * @brief Generate the initial gait sequence
     *
     * Creates a smooth transition from standing posture
     * to a stable cyclic gait.
     *
     * @param n_samples Number of samples per step
     */
    void generate_init_gait_sequence(short n_samples);

    /**
     * @brief Generate the cyclic gait sequence
     *
     * Produces the repeating walking pattern after initialization.
     *
     * @param n_samples Number of samples per step
     */
    void generate_gait_sequence(short n_samples);

    /**
     * @brief Number of samples per gait step
     */
    short total_samples_per_step;

    /**
     * @brief Initialization Cartesian trajectory table
     *
     * Dimensions:
     * [init_step][sample][leg][xyz]
     */
    float init_coord_list[3][25][4][3];

    /**
     * @brief Initialization PWM table
     *
     * Dimensions:
     * [init_step][sample][leg][servo]
     */
    short init_pwm_list[3][25][4][3];

    /**
     * @brief Cyclic gait Cartesian trajectory table
     *
     * Dimensions:
     * [step][sample][leg][xyz]
     */
    float coord_list[4][25][4][3];

    /**
     * @brief Cyclic gait PWM table
     *
     * Dimensions:
     * [step][sample][leg][servo]
     */
    short pwm_list[4][25][4][3];

  public:

    /**
     * @brief List of leg objects participating in the gait
     *
     * Order:
     * 0 = RF, 1 = LF, 2 = RB, 3 = LB
     */
    SpiderLeg* leg_list[4];

    /**
     * @brief Construct a Gait object
     *
     * @param leg_list Array of pointers to SpiderLeg objects
     */
    Gait(SpiderLeg* leg_list[4]);

    /**
     * @brief Initialize gait parameters
     *
     * @param direction      'x', 'y', or 't'
     * @param inv_direction  Reverse gait direction flag
     * @param step_length    Step length (linear or angular)
     * @param step_height    Maximum foot lift height
     * @param velocity       Desired gait velocity
     * @param freq           Controller update frequency (Hz)
     */
    void init(char direction, bool inv_direction, float step_length, float step_height, float velocity, float freq);

    /**
     * @brief Retrieve a Cartesian coordinate from the gait tables
     *
     * @param step   Gait step index
     * @param sample Sample index within the step
     * @param leg    Leg index
     * @param ii     Coordinate index (0=x, 1=y, 2=z)
     * @param init   True for initialization gait, false for cyclic gait
     *
     * @return Requested coordinate value
     */
    float get_coordinate_from_list(short step, short sample, short leg, short ii, bool init);

    /**
     * @brief Retrieve a PWM value from the gait tables
     *
     * @param step   Gait step index
     * @param sample Sample index within the step
     * @param leg    Leg index
     * @param ii     Servo index
     * @param init   True for initialization gait, false for cyclic gait
     *
     * @return PWM value
     */
    short get_pwm_from_list(short step, short sample, short leg, short ii, bool init);

    /**
     * @brief Get the number of samples per gait step
     */
    short get_sample_no();

    /**
     * @brief Convert a trajectory into angles and PWM values
     *
     * @param coordinates Foot trajectory [n][3]
     * @param step_no     Gait step index
     * @param leg_no      Leg index
     * @param n_samples   Number of samples
     * @param init        True for initialization gait
     */
    void write_data_to_lists(float coordinates[][3], short step_no, short leg_no, short n_samples, bool init);

    /**
     * @brief Generate a Cartesian foot offset from the initial pose
     *
     * @param leg_no Leg index
     * @param offset Normalized offset in gait phase [-0.5, 0.5]
     * @param result Output Cartesian coordinate [x, y, z]
     */
    void generate_coord_offset(short leg_no, float offset, float result[3]);

    /**
     * @brief Update gait velocity
     *
     * Recomputes sample count and regenerates gait tables.
     *
     * @param velocity Desired gait velocity
     */
    void set_velocity(float velocity);

    /**
     * @brief Get a human-readable gait name
     *
     * Format:
     * "+x", "-y", "+t", etc.
     *
     * @param name Output array of size 2
     */
    const char* get_name();
};

#endif
