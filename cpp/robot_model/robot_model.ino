// #include "FourBarLinkage.cpp"
#include "SpiderLeg.cpp"

int main() {
  // Example usage:
  FourBarLinkage linkage(20, 17, 30, 15, 90);
  SpiderLeg RFL(43.5, 42, 1, 1, "RFL", 66.5, -23, 30.5, 78.2, 131.5, 147.5, 90 - 9.5, 3.7);
  linkage.set_pwm_init(300, 1);
  double phi_value = 92;  // Set your desired phi value
  int pwm_result = linkage.calc_PWM(phi_value);
  double theta = linkage.calc_theta(phi_value) / M_PI * 180;
  double phi[3];
  RFL.get_actuator_angles(phi);
  double pos[3];
  RFL.forward_transform(phi[0],phi[1],phi[2], pos[0], pos[1], pos[2]);
  Serial.println(String("PWM value  ") + pwm_result + ". Will result in " + theta + " deg actuator theta angle.");
}


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  main();  // put your main code here, to run repeatedly:
}
