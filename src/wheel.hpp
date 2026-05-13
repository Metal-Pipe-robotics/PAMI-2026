#ifndef _WHEEL_HPP_ 
#define _WHEEL_HPP_

enum side_t {
  LEFT = -1,
  RIGHT = 1
};

void wheels_setpoint_relative(const float dist, const enum side_t side);
void wheels_init();
void wheels_setpoint(const float dist, const enum side_t side); 
void wheels_update(float dz);
void wheel_setpwm(const float pwm, const enum side_t side);
float wheels_get_rpm(const enum side_t side);
void wheel_setpwm(const float pwm, const enum side_t side);
void encoders_update();
void wheels_switch_off();
long wheels_get_ticks(const enum side_t side);
void odometry_get_pos(double *_x, double *_y, double *_z);
void wheels_setmode(int mode);
int is_cmd_finished();
void encoders_reset();

#endif /* _WHEEL_HPP_ */
