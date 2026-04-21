#ifndef _WHEEL_HPP_ 
#define _WHEEL_HPP_

enum side_t {
  LEFT = -1,
  RIGHT = 1
};

void wheels_init();
void wheels_setpoint(const float dist, const enum side_t side); 
void wheels_update();
void wheel_setpwm(const float pwm, const enum side_t side);
float wheels_get_rpm(const enum side_t side);
void wheel_setpwm(const float pwm, const enum side_t side);
void encoders_update();

#endif /* _WHEEL_HPP_ */
