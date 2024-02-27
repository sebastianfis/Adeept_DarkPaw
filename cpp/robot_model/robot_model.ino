#include "FourBarLinkage.h"
#include "math.h"
// #include "SpiderLeg.h"

FourBarLinkage linkage(42.5, 14.5, 38, 27.8, 90);


int main() {
  // Example usage:
  Serial.println(String("Starting main"));
  Serial.println(String("FourBarLinkage successfully initiated"));
  Serial.println(String(linkage.get_phi_0()));
  linkage.set_pwm_init(300, 1);
  double phi_value = 92;  // Set your desired phi value

  int pwm_result = linkage.calc_PWM(phi_value*M_PI/180);

  Serial.println(String("PWM calculated"));
  Serial.println(String(phi_value*M_PI/180));
  double theta = linkage.calc_theta(phi_value*M_PI/180) / M_PI * 180;
  Serial.println(String("PWM value ") + pwm_result + " will result in " + theta + " deg actuator theta angle.");
  // SpiderLeg RFL(43.5, 42, 1, 1, "RFL", 66.5, -23, 30.5, 78.2, 131.5, 147.5, 90 - 9.5, 3.7);
  // Serial.println(String("SpiderLeg successfully initiated"));
  // double phi[3];
  // RFL.get_actuator_angles(phi);
  // Serial.println(String("Acutator angles succesfully read"));
  // double pos[3];
  // RFL.forward_transform(phi[0],phi[1],phi[2], pos[0], pos[1], pos[2]);
  // Serial.println(String("Forward transfomr succesful!"));
  // Serial.println(String("actuator angles  ") + phi[0] + ", " + phi[1] + ", " + phi[2] + " will result in x_f =" + pos[0] + ", y_f=" + pos[1] + ", z_f=" + pos[2] + " foot position.");
  delay(1000);
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
