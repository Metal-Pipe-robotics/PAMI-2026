#ifndef TEST_ODOMETRY_HPP_ 
#define TEST_ODOMETRY_HPP_
#include "wheel.hpp"
#include "config.hpp"

void test_odometry() {
  wheels_switch_off();
  
  double x,y,z;
  long lticks, rticks;
  const long PRINT_INTERVAL_MS = 1000;
  const long start = millis();
  long now = millis();
  long enc_clock = now;
  long print_clock = now;
  Serial.printf("Time(s), x, y ,z, rticks, lticks\n\n");
  while (true) {
    now = millis();
    if (now - enc_clock >= WHEEL_UPDATE_INTERVAL_MS) {
      encoders_update();
      odometry_get_pos(&x, &y, &z);
      lticks = wheels_get_ticks(LEFT);
      rticks = wheels_get_ticks(RIGHT);
      enc_clock = now;
    }


    if (now - print_clock >= PRINT_INTERVAL_MS) {
      print_clock = now;
      Serial.printf("%3.3ld \t|  %2.3lf  %2.3lf  %2.3lf \t|  %6ld  %6ld\n", (now - start)*1e-3, x, y, z, lticks, rticks);
    }
  }
}

#endif /* TEST_ODOMETRY_HPP_ */
