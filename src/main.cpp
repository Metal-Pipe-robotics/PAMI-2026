#include <Arduino.h>
#include "wheel.hpp"
#include "config.hpp"
#include "tools.hpp"
#include "test_motor_simple.hpp"



long t_reverse_engine;
long t_stop_engine;
void setup() {
  Serial.begin(9600);
  Serial.printf("Begin setup\n");
  wheels_init();
  sleep(1);
  t_reverse_engine = millis() +10000;
  t_stop_engine = millis() +16000;
  wheels_setpoint(.5, RIGHT);
  wheels_setpoint(.5, LEFT);
  Serial.printf("Setup finished -> %ld %ld\n", millis(), t_reverse_engine);
}

unsigned long wheels_clock = 0;
void loop() {
  const long now = millis();
  // test_motor_simple_begin();
  // test_display_motor_speed();
  if (now - t_reverse_engine >= 0) {
    Serial.printf("Reversing (%ld %ld)\n", now, now-t_reverse_engine);
    wheels_setpoint(0., RIGHT);
    wheels_setpoint(0., LEFT);
    t_reverse_engine += 10e7;
  }
  // if (now - t_stop_engine >= 0) {
  //   wheels_setpoint(-0.2, RIGHT);
  //   wheels_setpoint(-0.2, LEFT);
  //   t_stop_engine += 10e7;
  // }
  if (now - wheels_clock >= WHEEL_UPDATE_INTERVAL_MS) {
      wheels_update();
      wheels_clock = now;
    }
}
