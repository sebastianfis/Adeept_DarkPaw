#ifndef FunctionTests_h
#define FunctionTests_h
#include "FourBarLinkage.h"
#include "SpiderLeg.h"
#include "Gait.h"
#include "RobotModel.h"

void test_linkage(float l_sg, float l_sa, float l_ab,  float l_gb, float phi_0);
void test_legs(SpiderLeg* leg_list[4]);
void test_pose(RobotModel *model, short pose_no);
void test_reset_step(RobotModel *model);
void test_gait(RobotModel *model, short gait_no);
void run_tests(RobotModel *model);
#endif