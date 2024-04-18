// #include "FourBarLinkage.h"
#include "Arduino.h"
#include "math.h"
#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"
#include "RobotModel.h"
#include "RobotController.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

// FourBarLinkage linkage(42.5, 14.5, 38, 27.8, 90);
SpiderLeg RF(1, 1, "RF");
SpiderLeg LF(1, -1, "LF");
SpiderLeg RB(-1, 1, "RB");
SpiderLeg LB(-1, -1, "LB");


Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

SpiderLeg *leg_list[] = {&RF, &LF, &RB, &LB};
RobotController robot_control(leg_list, &Serial, &pwm, &mpu);


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
//   float result[3];
  
//   for (int leg = 0 ; leg < 4 ; leg++) {
//     String leg_name = leg_list[leg]->get_name();
//     leg_list[leg]->get_init_pos(result);
//     Serial.println(String("Initial foot position of ") + leg_name + ": x_f = " + result[0] + ", y_f = " + result[1] + ", z_f = " + result[2]);
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

// void test_pose(RobotModel *model, short pose_no) {

//   Serial.println(String("testing pose ") + model->pose_list[pose_no]->get_name());
//   for (short leg = 0; leg < 4; ++leg) {
//     float coords[3];
//     model->leg_list[leg]->get_cur_pos(coords);
//     Serial.println(String("foot position of ") + model->leg_list[leg]->get_name() + ": (" + coords[0] + ", " + coords[1] + ", " + coords[2] + ")");
//   }

//   Serial.println(String("movement goal successfully updated"));
//   float target[4][3];
//   model->pose_list[pose_no]->get_movement_goal(target);

//   for (short leg = 0; leg < 4; ++leg){
//     Serial.println(String("movement goal for ") + model->leg_list[leg]->get_name() + ": (" + target[leg][0] + ", " + target[leg][1]  + ", " + target[leg][2]  + ")");
//     Serial.flush();
//     }

//   for (short leg = 0; leg < 4; ++leg){
//     for (short sample = 0; sample < model->pose_list[pose_no]->get_sample_no(); ++sample) {
//       Serial.println(String("coordinate sample of ") + model->leg_list[leg]->get_name() + ": (" + model->pose_list[pose_no]->get_coordinate_from_list(sample, leg, 0) + ", " + model->pose_list[pose_no]->get_coordinate_from_list(sample, leg, 1) + 
//                                                                                           ", " + model->pose_list[pose_no]->get_coordinate_from_list(sample, leg, 2) + ")");
//       Serial.flush();
//     }
//     for (short sample = 0; sample < model->pose_list[pose_no]->get_sample_no(); ++sample) {
//       Serial.println(String("PWM sample of ") + model->leg_list[leg]->get_name() + ": (" + model->pose_list[pose_no]->get_pwm_from_list(sample, leg, 0) + ", " + model->pose_list[pose_no]->get_pwm_from_list(sample, leg, 1) + 
//                                                                                     ", " + model->pose_list[pose_no]->get_pwm_from_list(sample, leg, 2) + ")");
//       Serial.flush();
//     }
  
//   }
//   float z_0;
//   for (short leg = 0; leg < 4; ++leg){
//     model->leg_list[leg]->update_cur_phi(target[leg][0],target[leg][1],target[leg][2]);
//     float coord[3];
//     model->leg_list[leg]->get_cur_pos(coord);
//     z_0 += coord[2];
//   }
//   z_0 = z_0 /4;
//   float angles[2];
//   model->get_body_angles(angles);
//   Serial.println(String(" "));
//   Serial.println(String("calcluated body angles: Theta_x = ") + angles[0] + ", Theta_y = " + angles[1] + ". Evaluted z_0 = " + z_0);
//   Serial.println(String(" "));
// }

void test_reset_step(RobotModel *model) {
  Serial.println(String("testing reset step"));
  float coords[3];
  model->leg_list[0]->get_cur_pos(coords);
  model->leg_list[0]->update_cur_phi(coords[0]+20, coords[1], coords[2]);
  model->leg_list[1]->get_cur_pos(coords);
  model->leg_list[1]->update_cur_phi(coords[0]-20, coords[1], coords[2]);
  model->leg_list[2]->get_cur_pos(coords);
  model->leg_list[2]->update_cur_phi(coords[0]+5, coords[1], coords[2]);
  model->leg_list[3]->get_cur_pos(coords);
  model->leg_list[3]->update_cur_phi(coords[0]-5, coords[1], coords[2]-10);
  model->calc_reset_step();
  for (short leg = 0; leg < 4; ++leg) {
    float coords[3];
    model->leg_list[leg]->get_cur_pos(coords);
    Serial.println(String("foot position of ") + model->leg_list[leg]->get_name() + ": (" + coords[0] + ", " + coords[1] + ", " + coords[2] + ")");
  }
  Serial.println(String(" "));
  for (short step = 0; step < 5; step++) {
    Serial.println(String("reset step no ") + step + ":");
    for (short leg = 0; leg < 4; ++leg){
      for (short sample = 0; sample < 8; ++sample) {
        Serial.println(String("coordinate sample of ") + model->leg_list[leg]->get_name() + ": (" + model->get_reset_coord(step, sample, leg, 0) + ", " +
                                                                                                    model->get_reset_coord(step, sample, leg, 1) + ", " +
                                                                                                    model->get_reset_coord(step, sample, leg, 2) + ")");
        Serial.flush();
      }
      for (short sample = 0; sample < 8; ++sample) {
        Serial.println(String("PWM sample of ") + model->leg_list[leg]->get_name() + ": (" + model->get_reset_pwm(step, sample, leg, 0) + ", " +
                                                                                            model->get_reset_pwm(step, sample, leg, 1) + ", " +
                                                                                            model->get_reset_pwm(step, sample, leg, 2) + ")");
        Serial.flush();
      }
    }
    Serial.println(String(" "));
  }
  
}

// void test_gait(RobotModel *model, short gait_no) { 
//   char name[2];
//   model->gait_list[gait_no]->get_name(name);
//   short total_samples = model->gait_list[gait_no]->get_sample_no();
//   Serial.println(String("gait ") + name + " has " + total_samples + " samples/step");
//   for (short init_step=0; init_step < 3; init_step++) {
//     Serial.println(String("init step no ") + init_step + ":");
//     for (short leg = 0; leg < 4; ++leg){
//       for (short sample = 0; sample < total_samples; ++sample) {
//         Serial.println(String("coordinate sample of ") + model->leg_list[leg]->get_name() + ": (" + model->gait_list[gait_no]->get_coordinate_from_list(init_step, sample, leg, 0, true) + ", " +
//                                                                                                 model->gait_list[gait_no]->get_coordinate_from_list(init_step, sample, leg, 1, true) + ", " +
//                                                                                                 model->gait_list[gait_no]->get_coordinate_from_list(init_step, sample, leg, 2, true) + ")");
//         Serial.flush();
//       }
//       for (short sample = 0; sample < total_samples; ++sample) {
//         Serial.println(String("PWM sample of ") + model->leg_list[leg]->get_name() + ": (" + model->gait_list[gait_no]->get_pwm_from_list(init_step, sample, leg, 0, true) + ", " +
//                                                                                         model->gait_list[gait_no]->get_pwm_from_list(init_step, sample, leg, 1, true) + ", " +
//                                                                                         model->gait_list[gait_no]->get_pwm_from_list(init_step, sample, leg, 2, true) + ")");
//         Serial.flush();
//       }
//     }
//   }
//   for (short step=0; step < 4; step++) {
//     Serial.println(String("step no ") + step + ":");
//     for (short leg = 0; leg < 4; ++leg){
//       for (short sample = 0; sample < total_samples; ++sample) {
//         Serial.println(String("coordinate sample of ") + model->leg_list[leg]->get_name() + ": (" + model->gait_list[gait_no]->get_coordinate_from_list(step, sample, leg, 0, false) + ", " +
//                                                                                                 model->gait_list[gait_no]->get_coordinate_from_list(step, sample, leg, 1, false) + ", " +
//                                                                                                 model->gait_list[gait_no]->get_coordinate_from_list(step, sample, leg, 2, false) + ")");
//         Serial.flush();
//       }
//       for (short sample = 0; sample < total_samples; ++sample) {
//         Serial.println(String("PWM sample of ") + model->leg_list[leg]->get_name() + ": (" + model->gait_list[gait_no]->get_pwm_from_list(step, sample, leg, 0, false) + ", " +
//                                                                                         model->gait_list[gait_no]->get_pwm_from_list(step, sample, leg, 1, false) + ", " +
//                                                                                         model->gait_list[gait_no]->get_pwm_from_list(step, sample, leg, 2, false) + ")");
//         Serial.flush();
//       }
//     }
//   }
//   Serial.println(String(" "));
// }


int main() {

  // test_linkage(); //success
  // test_legs(); //success
  // test_pose(); // success
  // test_coord_offset(0); // success
  // test_coord_offset(1); // success
  // test_coord_offset(2); // success
  // test_coord_offset(3); // success

  // test_pose(&robot_control.robot_model, 0); // success
  // delay(1000);
  // test_pose(&robot_control.robot_model, 1); // success
  // delay(1000);
  // test_pose(&robot_control.robot_model, 3); // success
  // delay(1000);
  // test_pose(&robot_control.robot_model, 5); // success
  // delay(1000);

  test_reset_step(&robot_control.robot_model);

  // test_gait(&robot_control.robot_model, 0); // success
  // delay(1000);
  // test_gait(&robot_control.robot_model, 2); // success
  // delay(1000);
  // test_gait(&robot_control.robot_model, 4); // success
  // delay(1000);
  // robot_model.set_velocity(80); //success
  // Serial.println(String(""));
  // Serial.println(String("Velocity changed"));
  // Serial.println(String(""));
  // test_gait(&robot_model, 0); //success
  // delay(1000);
  // test_gait(&robot_model, 2); //success
  // delay(1000);  

}


void setup() {
  robot_control.init();
  //setup_gaits();

  main();
  // put your setup code here, to run once:
}

void loop() {
    // put your main code here, to run repeatedly:
}
