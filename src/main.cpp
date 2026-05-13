#include <Arduino.h>
#include "vlx.h"
#include "pins.hpp"
#include "wheel.hpp"
#include "config.hpp"
#include "tools.hpp"
#include "test_motor_simple.hpp"
#include "test_odometry.hpp"
#include "waypoint.h"
#include "target_handler.h"
#include "servo.h"

void vTaskEncoders(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(ENCODERS_UPDATE_INTERVAL_MS);

    for (;;) {
        encoders_update();   // reads hardware — no shared state, no lock needed

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskWheels(void *pvParameters) {
    VL53L1X_task();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    double x, y, z;
    odometry_get_pos(&x, &y, &z);
    exec_next_cmd(x, y, z);
    const TickType_t xPeriod  = pdMS_TO_TICKS(WHEEL_UPDATE_INTERVAL_MS);

    for (;;) {
    VL53L1X_task();
        odometry_get_pos(&x, &y, &z);
        if (is_cmd_finished() || is_cmd_finished_pos(x, y, z))
          exec_next_cmd(x, y, z);
        const float dz = get_angle_to_target(x, y, z);
        wheels_update(dz);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskPrintAndLED(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(300);
    bool ledState = true;

    for (;;) {
        double x, y, z;
        odometry_get_pos(&x, &y, &z);
        
        Serial.printf("%3.3lf   %3.3lf   %3.3lf\n", x, y, z * 180.0 / M_PI);

        ledState = !ledState;
        digitalWrite(LEDBLUEP, ledState ? HIGH : LOW);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

int distance = 0;
void vTaskVlx(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(300);
  for (;;) {
    distance = VL53L1X_task();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void wait_for_tirette() {
  while (digitalRead(TIRETTE_PIN) == LOW) {
    vTaskDelay(100);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.printf("Begin setup\n");
  wheels_init();
  waypoint_init_all();
  def_targets();
  
  VL53L1X_setup();
  sleep(1);
  // wheels_setpoint(1.6, RIGHT);
  // wheels_setpoint(1.6, LEFT);
  wheels_update(0.f);
  pinMode(LEDBLUEP, OUTPUT);
  pinMode(LEDWHITEP, OUTPUT);
  pinMode(TIRETTE_PIN, INPUT_PULLUP);
  digitalWrite(LEDBLUEP, HIGH);
  digitalWrite(LEDWHITEP, LOW);
  wait_for_tirette();
  encoders_reset();
  xTaskCreate(vTaskEncoders,    "Encoders",  2048*4, NULL, 3, NULL);
  xTaskCreate(vTaskWheels,      "Wheels",    4096*4, NULL, 3, NULL);
  xTaskCreate(vTaskPrintAndLED, "Print",     2048*4, NULL, 1, NULL);
  vTaskDelay(1000);
  // servo_start();
}

void loop() {
  vTaskDelete(NULL);
  // test_odometry();
  // test_motor_simple_begin();
  // test_display_motor_speed();
 }
