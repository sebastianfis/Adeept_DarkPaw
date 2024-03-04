// #include "FourBarLinkage.h"
#include "math.h"
#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

// FourBarLinkage linkage(42.5, 14.5, 38, 27.8, 90);
SpiderLeg RF(1, 1, "RF");
SpiderLeg LF(1, -1, "LF");
SpiderLeg RB(-1, 1, "RB");
SpiderLeg LB(-1, -1, "LB");

SpiderLeg *leg_list[] = {&RF, &LF, &RB, &LB};

const short pose_n = 6;
float pose_goal[4][3];
Pose lift_LFL(leg_list, pose_goal, pose_n, "LFL");

// void test_linkage() {
//   Serial.println(String("FourBarLinkage successfully initiated"));
//   Serial.println(String("phi_0 is: ")+ linkage.get_phi_0()/ M_PI * 180);
//   linkage.set_pwm_init(300, 1);
//   float phi_value = 92;  // Set your desired phi value
//   short pwm_result = linkage.calc_PWM(phi_value*M_PI/180);
//   Serial.println(String("PWM calculated. Phi actuator angle of ") + phi_value + " deg equals a PWM value of " + pwm_result);
//   float theta = linkage.calc_theta(phi_value*M_PI/180);
//   float phi_r = linkage.calc_phi(theta);
//   Serial.println(String("PWM value ") + pwm_result + " will result in  " + theta / M_PI * 180 + " deg actuator theta angle. Backward transform gives phi = " + phi_r/ M_PI * 180 + " deg.");
// }

// void test_legs() {
//   float phi[3];
//   float pos[3];
//   float phi_ret[3];
  
//   for (int leg = 0 ; leg < 4 ; leg++) {
//     String leg_name = leg_list[leg]->get_name();
//     Serial.println(String("SpiderLeg ") + leg_name + " successfully initiated");
//     leg_list[leg]->get_actuator_angles(phi);
//     Serial.println(String("Actuator angles succesfully read"));
//     leg_list[leg]->forward_transform(phi[0], phi[1], phi[2], &pos[0], &pos[1], &pos[2]);
//     Serial.println(String("Forward transform succesful!"));
//     Serial.println(String("actuator angles for ") + leg_name + ": " + phi[0] / M_PI * 180.0 + ", " + phi[1]  / M_PI * 180.0 + ", " + phi[2]  / M_PI * 180.0 + " will result in x_f = " + pos[0] + ", y_f = " + pos[1] + ", z_f = " + pos[2] + " foot position.");
//     leg_list[leg]->backward_transform(pos[0], pos[1], pos[2], &phi_ret[0], &phi_ret[1], &phi_ret[2]);
//     Serial.println(String("Backward transform succesful!"));
//     Serial.println(String("foot position of ") + leg_name + ": x_f = " + pos[0] + ", y_f = " + pos[1] + ", z_f = " + pos[2] + " correspont to actuator angles of "+ phi_ret[0] / M_PI * 180.0 + ", " + phi_ret[1]  / M_PI * 180.0 + ", " + phi_ret[2]  / M_PI * 180.0);
//   }
// }

void test_pose() {
  for (short leg = 0; leg < 4; ++leg) {
    float coords[3];
    leg_list[leg]->get_cur_pos(coords);
    Serial.println(String("foot position of ") + leg_list[leg]->get_name() + ": (" + coords[0] + ", " + coords[1] + ", " + coords[2] + ")");
    for (short ii = 0; ii < 3; ++ii) {
      pose_goal[leg][ii] = coords[ii];
    }
  }
  pose_goal[1][2] -= 12;
  
  lift_LFL.set_movement_goal(pose_goal);
  Serial.println(String("Movement goal successfully updated"));
  float target[4][3];
  lift_LFL.get_movement_goal(target);
  for (short leg = 0; leg < 4; ++leg){
    Serial.println(String("movement goal for ") + leg_list[leg]->get_name() + ": (" + target[leg][0] + ", " + target[leg][1]  + ", " + target[leg][2]  + ")");
    Serial.flush();
    }
  float coord_list[pose_n][4][3];
  short pwm_list[pose_n][4][3];
  lift_LFL.calc_pose_lists(coord_list, pwm_list);
  Serial.println(String(F("pose calculated successfully!")));
  for (short leg = 0; leg < 4; ++leg){
    for (short sample = 0; sample < pose_n; ++sample) {
      Serial.println(String("coordinate sample of ") + leg_list[leg]->get_name() + ": (" + coord_list[sample][leg][0] + ", " + coord_list[sample][leg][1] + ", " + coord_list[sample][leg][2] + ")");
      Serial.flush();
    }
    for (short sample = 0; sample < 6; ++sample) {
      Serial.println(String("PWM sample of ") + leg_list[leg]->get_name() + ": (" + pwm_list[sample][leg][0] + ", " + pwm_list[sample][leg][1] + ", " + pwm_list[sample][leg][2] + ")");
      Serial.flush();
    }
  }

}

int main() {

 // test_linkage();
 // test_legs();
  test_pose();
  delay(5000);
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n Starting...\n");
  Serial.flush();
  // put your setup code here, to run once:
}

void loop() {
  main();  // put your main code here, to run repeatedly:
}
