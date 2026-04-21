#ifndef _TEST_MOTOR_SPEED_HPP_
#define _TEST_MOTOR_SPEED_HPP_
#include <Arduino.h>
#include "wheel.hpp"

static void test_motor_speed_begin() {
  Serial.printf("Begining '%s'\n", __FILE__);

  wheel_setpwm(0.f, LEFT);
  wheel_setpwm(0.f, RIGHT);
  sleep(1);

  int pwm;
  for (pwm=1; pwm<256; pwm++) {
    wheel_setpwm(-pwm, LEFT);
    wheel_setpwm(-pwm, RIGHT);
    encoders_update();
    usleep(250000);
    encoders_update();
    const float lrpm = wheels_get_rpm(LEFT);
    const float rrpm = wheels_get_rpm(RIGHT);
    Serial.printf(">LEFT_increase:%d:%f|xy\n", -pwm, fabs(lrpm));
    Serial.printf(">RIGHT_increase:%d:%f|xy\n", -pwm, fabs(rrpm));
  }

  wheel_setpwm(0.f, LEFT);
  wheel_setpwm(0.f, RIGHT);
  sleep(2);

  for (pwm=0; pwm<256; pwm++) {
    wheel_setpwm(pwm, LEFT);
    wheel_setpwm(pwm, RIGHT);
    encoders_update();
    usleep(250000);
    encoders_update();
    const float lrpm = wheels_get_rpm(LEFT);
    const float rrpm = wheels_get_rpm(RIGHT);
    Serial.printf(">LEFT_increase:%d:%f|xy\n", pwm, fabs(lrpm));
    Serial.printf(">RIGHT_increase:%d:%f|xy\n", pwm, fabs(rrpm));
  }

  wheel_setpwm(0.f, LEFT);
  wheel_setpwm(0.f, RIGHT);
  sleep(2);

  for (pwm=255; pwm>0; pwm--) {
    wheel_setpwm(-pwm, LEFT);
    wheel_setpwm(-pwm, RIGHT);
    encoders_update();
    usleep(250000);
    encoders_update();
    const float lrpm = wheels_get_rpm(LEFT);
    const float rrpm = wheels_get_rpm(RIGHT);
    Serial.printf(">LEFT_decrease:%d:%f|xy\n", -pwm, fabs(lrpm));
    Serial.printf(">RIGHT_decrease:%d:%f|xy\n", -pwm, fabs(rrpm));
  }

  wheel_setpwm(0.f, LEFT);
  wheel_setpwm(0.f, RIGHT);
  sleep(2);

  for (pwm=255; pwm>=0; pwm--) {
    wheel_setpwm(pwm, LEFT);
    wheel_setpwm(pwm, RIGHT);
    encoders_update();
    usleep(250000);
    encoders_update();
    const float lrpm = wheels_get_rpm(LEFT);
    const float rrpm = wheels_get_rpm(RIGHT);
    Serial.printf(">LEFT_decrease:%d:%f|xy\n", pwm, fabs(lrpm));
    Serial.printf(">RIGHT_decrease:%d:%f|xy\n", pwm, fabs(rrpm));
  }

  wheel_setpwm(0.f, LEFT);
  wheel_setpwm(0.f, RIGHT);
  Serial.printf(">TEST_FINISHED\n");
  while(1);
}


void test_display_motor_speed(){
  encoders_update();
  usleep(250000);
  encoders_update();
  const float lrpm = wheels_get_rpm(LEFT);
  const float rrpm = wheels_get_rpm(RIGHT);
  // Serial.printf(">LEFT:%f\n", fabs(lrpm));
  Serial.printf("%f\n", fabs(rrpm));


}

#endif /* _TEST_MOTOR_SPEED_HPP_ */
