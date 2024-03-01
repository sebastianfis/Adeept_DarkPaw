#include "FourBarLinkage.h"
#include "math.h"
#include "SpiderLeg.h"

FourBarLinkage linkage(42.5, 14.5, 38, 27.8, 90);

SpiderLeg leg_list[] = {
  SpiderLeg(1, 1, "RFL"), 
  SpiderLeg(1, -1, "LFL"), 
  SpiderLeg(-1, 1, "RBL"), 
  SpiderLeg(-1, -1, "LBL")
};

void test_linkage() {
  Serial.println(String("FourBarLinkage successfully initiated"));
  Serial.println(String("phi_0 is: ")+ linkage.get_phi_0()/ M_PI * 180);
  linkage.set_pwm_init(300, 1);
  float phi_value = 92;  // Set your desired phi value
  short pwm_result = linkage.calc_PWM(phi_value*M_PI/180);
  Serial.println(String("PWM calculated. Phi actuator angle of ") + phi_value + " deg equals a PWM value of " + pwm_result);
  float theta = linkage.calc_theta(phi_value*M_PI/180);
  float phi_r = linkage.calc_phi(theta);
  Serial.println(String("PWM value ") + pwm_result + " will result in  " + theta / M_PI * 180 + " deg actuator theta angle. Backward transform gives phi = " + phi_r/ M_PI * 180 + " deg.");
}

void test_legs() {
  float phi[3];
  float pos[3];
  float phi_ret[3];
  
  for (int leg = 0 ; leg < 4 ; leg++) {
    String leg_name = leg_list[leg].get_name();
    Serial.println(String("SpiderLeg ") + leg_name + " successfully initiated");
    leg_list[leg].get_actuator_angles(phi);
    Serial.println(String("Actuator angles succesfully read"));
    leg_list[leg].forward_transform(phi[0], phi[1], phi[2], &pos[0], &pos[1], &pos[2]);
    Serial.println(String("Forward transform succesful!"));
    Serial.println(String("actuator angles for ") + leg_name + ": " + phi[0] / M_PI * 180.0 + ", " + phi[1]  / M_PI * 180.0 + ", " + phi[2]  / M_PI * 180.0 + " will result in x_f = " + pos[0] + ", y_f = " + pos[1] + ", z_f = " + pos[2] + " foot position.");
    leg_list[leg].backward_transform(pos[0], pos[1], pos[2], &phi_ret[0], &phi_ret[1], &phi_ret[2]);
    Serial.println(String("Backward transform succesful!"));
    Serial.println(String("foot position of ") + leg_name + ": x_f = " + pos[0] + ", y_f = " + pos[1] + ", z_f = " + pos[2] + " correspont to actuator angles of "+ phi_ret[0] / M_PI * 180.0 + ", " + phi_ret[1]  / M_PI * 180.0 + ", " + phi_ret[2]  / M_PI * 180.0);
  }
}

int main() {

  test_linkage();
  test_legs();
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
