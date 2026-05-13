#include <Arduino.h>
#include "config.hpp"
#include <math.h>
#include "wheel.hpp"
#include "waypoint.h"
#include "tools.hpp"

static struct Pose robot_target_pos; 

static int dirmode = 0;
void def_targets() {
// waypoint_add_pose( (struct Pose) {
//     0.5f,
//     0.f,
//     0.f,
//     0.05f
//   });
  waypoint_add_pose( (struct Pose) {
    0.5f,
    0.f,
    M_PI,
    0.05f
  });
// waypoint_add_pose( (struct Pose) {
//     0.0f,
//     0.f,
//     M_PI,
//     0.05f
//   });
}

float get_angle_to_target(float x, float y, float z) {
  if (robot_target_pos.x == x && robot_target_pos.y == y)
    return 0.0f;

  float dx = robot_target_pos.x - x;
  float dy = robot_target_pos.y - y;

  if (dx == 0)
    return -normalizeAngle((-dy > 0 ? M_PI / 2 : -M_PI / 2) - z);

  return -normalizeAngle(atan2(-dy, dx) - z);
}
// float get_angle_to_target(float x, float y, float z) {
//   if (robot_target_pos.x == x && robot_target_pos.y == y)
//     return -z;
//   if (robot_target_pos.x == x)
//     return M_PI / 2 - z;
//   return normalizeAngle(atan((robot_target_pos.y - y) / (robot_target_pos.x - x)) - z);
// }

void exec_next_cmd(float x, float y, float z) {
  Serial.printf("next cmd\n");
  struct Pose next_target = waypoint_pop_pose();
  if (next_target.x < 0.f ) { /* No new target */
    wheels_switch_off();
    Serial.printf("end \n");
    return;
  }
  robot_target_pos = next_target;

  // const float dz = get_angle_to_target(x, y, z );
  const float dz = robot_target_pos.theta - z;

  Serial.printf(">%f\n", dz);
  if (fabs(dz) > M_PI / 40) { // rotation
    dirmode = 1;
    const float circle_ratio = dz / (2*M_PI);
    const float sdist = WHEELS_SPACING * M_PI * circle_ratio;
    wheels_setmode(1);
  Serial.printf(">>%f\n", sdist);
    wheels_setpoint(-sdist, LEFT);
    wheels_setpoint(sdist, RIGHT);
  } else {
    dirmode = 0;
    const float distance = sqrtf((robot_target_pos.x - x) * (robot_target_pos.x - x) + (robot_target_pos.y - y) * (robot_target_pos.y - y));
    wheels_setmode(0);
    wheels_setpoint(distance, LEFT);
    wheels_setpoint(distance, RIGHT);
  }
}

int is_cmd_finished_pos(float x, float y, float z) {
  if (dirmode == 1) {
    const float dz = fabs(robot_target_pos.theta - z);
    // Serial.printf("%f\n", dz);
    if (dz <  M_PI / 80.)
      return 1;
  } else if (dirmode == 0) {
    const float distance = (robot_target_pos.x - x) * (robot_target_pos.x - x) + (robot_target_pos.y - y) * (robot_target_pos.y - y);
    if (distance < 0.001)
      return 1;
  }
  return 0;
}

/* 
 * radius cercle = (ecart_whel * M_PI) / (z / (2*M_PI))
 */


