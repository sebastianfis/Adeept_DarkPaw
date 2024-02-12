// #include "FourBarLinkage.cpp"
#include "SpiderLeg.cpp"

int main() {
  // Example usage:
  FourBarLinkage linkage(20, 17, 30, 15, 90);
  FourBarLinkage linkage2(20, 17, 30, 15, 90);
  FourBarLinkage linkage3(20, 17, 30, 15, 90);
  FourBarLinkage linkage4(20, 17, 30, 15, 90);
  FourBarLinkage linkage5(20, 17, 30, 15, 90);
  FourBarLinkage linkage6(20, 17, 30, 15, 90);
  FourBarLinkage linkage7(20, 17, 30, 15, 90);
  FourBarLinkage linkage8(20, 17, 30, 15, 90);
  FourBarLinkage linkage9(20, 17, 30, 15, 90);
  FourBarLinkage linkage10(20, 17, 30, 15, 90);
  FourBarLinkage linkage11(20, 17, 30, 15, 90);
  FourBarLinkage linkage12(20, 17, 30, 15, 90);
  linkage.set_pwm_init(300, 1);
  double phi_value = 92;  // Set your desired phi value
  int pwm_result = linkage.calc_PWM(phi_value);
  double theta = linkage.calc_theta(phi_value) / M_PI * 180;
  Serial.println(String("PWM value  ") + pwm_result + ". Will result in " + theta + " deg actuator theta angle.");
}


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  main();  // put your main code here, to run repeatedly:
}
