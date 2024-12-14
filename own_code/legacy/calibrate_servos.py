# This code is legacy only. All implementaions herein have been ported to C++ to run directly on the ESP

import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

LF1_port = 0
LF2_port = 1
LF3_port = 2

RF1_port = 6
RF2_port = 7
RF3_port = 8

LB1_port = 3
LB2_port = 4
LB3_port = 5

RB1_port = 9
RB2_port = 10
RB3_port = 11

P_port = 12
T_port = 13

LF1_init_pwm = 305
LF2_init_pwm = 325
LF3_init_pwm = 330

RF1_init_pwm = 295
RF2_init_pwm = 285
RF3_init_pwm = 290

LB1_init_pwm = 295
LB2_init_pwm = 300
LB3_init_pwm = 285

RB1_init_pwm = 365
RB2_init_pwm = 340
RB3_init_pwm = 345

P_init_pwm = 300
T_init_pwm = 300


def set_pwm(pwm_driver, port, value):
    pwm_driver.set_pwm(port, 0, value)
    print("init_pwm of port " + str(port) + " is " + str(value))


def set_init_pwm(pwm_driver):
    pwm_driver.set_pwm(LF1_port, 0, LF1_init_pwm)
    pwm_driver.set_pwm(LF2_port, 0, LF2_init_pwm)
    pwm_driver.set_pwm(LF3_port, 0, LF3_init_pwm)
    pwm_driver.set_pwm(LB1_port, 0, LB1_init_pwm)
    pwm_driver.set_pwm(LB2_port, 0, LB2_init_pwm)
    pwm_driver.set_pwm(LB3_port, 0, LB3_init_pwm)
    pwm_driver.set_pwm(RF1_port, 0, RF1_init_pwm)
    pwm_driver.set_pwm(RF2_port, 0, RF2_init_pwm)
    pwm_driver.set_pwm(RF3_port, 0, RF3_init_pwm)
    pwm_driver.set_pwm(RB1_port, 0, RB1_init_pwm)
    pwm_driver.set_pwm(RB2_port, 0, RB2_init_pwm)
    pwm_driver.set_pwm(RB3_port, 0, RB3_init_pwm)
    print("all ports set to initial init pwm")


def main():
    while True:
        port = input("Please select a PWM port(0-11). Use A to set all ports. Use I to set all ports to original "
                     "init PWM. Press Q to exit")
        if port == 'Q' or port == 'q':
            break
        if port == 'I' or port == 'i':
            set_init_pwm(pwm)
        else:
            value = input("Please set a PWM calibration value(330 +/-50). Press Q to exit")
            if value == 'Q' or value == 'q':
                break
            if port == 'A' or port == 'a':
                for p in range(12):
                    set_pwm(pwm, int(p), int(value))
            else:
                set_pwm(pwm, int(port), int(value))


if __name__ == '__main__':
    main()
