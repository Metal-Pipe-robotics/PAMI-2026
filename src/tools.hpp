#ifndef _TOOLS_H_

#include <math.h>
#include "config.hpp"



static inline float constrainf(const float x, const float min, const float max) {
  if (x <= min)
    return min;
  else if (x >= max)
    return max;
  return x;
}

static inline float ticks_to_rotations(const long ticks) {
  return ((float) ticks) / TICKS_PER_REVOLUTION;
}

static inline float rotations_to_distance(const float rotations) {
  return rotations * 2 * M_PI * WHEEL_RADIUS;
}

static inline float ticks_to_distance(const long ticks) {
  return rotations_to_distance(ticks_to_rotations(ticks));
}

static inline long rotations_to_ticks(const float rotations) {
  return rotations * TICKS_PER_REVOLUTION;
}

static inline float distance_to_rotations(const float distance) {
  return distance / (2 * M_PI * WHEEL_RADIUS);
}

static inline long distance_to_ticks(const float distance) {
  return rotations_to_ticks(distance_to_rotations(distance));
}

static inline float normalizeAngle(float angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

#endif /* _TOOLS_H_ */
