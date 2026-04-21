#ifndef _TEST_MOTOR_SIMPLE_HPP_
#define _TEST_MOTOR_SIMPLE_HPP_
#include <Arduino.h>
#include "wheel.hpp"


static void _test_motor_simple(const char *tested_motor, bool is_forward) {
  const int pwms[5] = {0, 65, 128};
  const int sign = is_forward ? 1 : -1;
  Serial.printf("\n==========================================\n");
  Serial.printf("\n%s motor test. Direction : %s\n", tested_motor, is_forward ? "forward" : "backward");
  int i;
  for (i=0; i<5; i++) {
    const int pwm = pwms[i];
    Serial.printf("\nTesting speed: \t %.1f %% \n", 100. * pwm / 255.);
    if (!strcmp(tested_motor, "Left")) {
      wheel_setpwm(pwm * sign, LEFT);
      wheel_setpwm(0, RIGHT);
    } else {
      wheel_setpwm(0, LEFT);
      wheel_setpwm(pwm * sign, RIGHT);
    }
    encoders_update();
    sleep(3);
    encoders_update();
    const float lrpm = wheels_get_rpm(LEFT);
    const float rrpm = wheels_get_rpm(RIGHT);
    Serial.printf("Left rpm read : %.3f\n", lrpm);
    Serial.printf("Right rpm read : %.3f\n", rrpm);
  }

  wheel_setpwm(0.f, LEFT);
  wheel_setpwm(0.f, RIGHT);
  sleep(1);
}

static void test_motor_simple_begin() {
  Serial.printf("Begining '%s'\n", __FILE__);
  _test_motor_simple("Left", true);
  _test_motor_simple("Right", true);
  _test_motor_simple("Left", false);
  _test_motor_simple("Right", false);

  Serial.printf("\nTest ended.\n");
  while(1) sleep(1);
  }



#endif /* _TEST_MOTOR_SIMPLE_HPP_ */
