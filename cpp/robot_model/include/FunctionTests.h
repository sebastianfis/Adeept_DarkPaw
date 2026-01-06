#ifndef FunctionTests_h
#define FunctionTests_h
#include "FourBarLinkage.h"
#include "SpiderLeg.h"
#include "Gait.h"
#include "RobotModel.h"

/**
 * @brief Tests the FourBarLinkage class
 *
 * Initializes a linkage, prints initial values, calculates PWM for a target phi,
 * and performs forward/backward angle calculations.
 *
 * @param l_sg Length of SG link
 * @param l_sa Length of SA link
 * @param l_ab Length of AB link
 * @param l_gb Length of GB link
 * @param phi_0 Initial phi angle in degrees
 */
void test_linkage(float l_sg, float l_sa, float l_ab,  float l_gb, float phi_0);
/**
 * @brief Tests SpiderLeg objects
 *
 * Reads initial positions, actuator angles, performs forward/backward transformations,
 * and prints results for each leg.
 *
 * @param leg_list Array of 4 SpiderLeg pointers
 */
void test_legs(SpiderLeg* leg_list[4]);
/**
 * @brief Tests a specific pose in the RobotModel
 *
 * Prints current foot positions, movement goals, sample coordinates, PWM samples,
 * and calculates body angles.
 *
 * @param model Pointer to RobotModel
 * @param pose_no Index of the pose to test
 */
void test_pose(RobotModel *model, short pose_no);
/**
 * @brief Tests reset step calculation in RobotModel
 *
 * Updates leg positions, calculates reset steps, and prints coordinates and PWM for all samples.
 *
 * @param model Pointer to RobotModel
 */
void test_reset_step(RobotModel *model);
/**
 * @brief Tests a gait sequence in the RobotModel
 *
 * Prints coordinates and PWM samples for initialization and gait steps.
 *
 * @param model Pointer to RobotModel
 * @param gait_no Index of the gait to test
 */
void test_gait(RobotModel *model, short gait_no);
/**
 * @brief Runs a full suite of tests on the robot model
 *
 * Executes linkage, leg, pose, reset step, gait, and velocity tests sequentially.
 *
 * @param model Pointer to RobotModel
 */
void run_tests(RobotModel *model);
#endif