#include <Arduino.h>
#include "pins.hpp"
#include "wheel.hpp"
#include "config.hpp"
#include "tools.hpp"
#include "test_motor_simple.hpp"
#include "test_odometry.hpp"


void vTaskEncoders(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(ENCODERS_UPDATE_INTERVAL_MS);

    for (;;) {
        encoders_update();   // reads hardware — no shared state, no lock needed

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskWheels(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod  = pdMS_TO_TICKS(WHEEL_UPDATE_INTERVAL_MS);

    for (;;) {
        wheels_update();

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

void setup() {
  Serial.begin(9600);
  Serial.printf("Begin setup\n");
  wheels_init();
  sleep(1);
  wheels_setpoint(1.6, RIGHT);
  wheels_setpoint(1.6, LEFT);
  wheels_update();
  pinMode(LEDBLUEP, OUTPUT);
  pinMode(LEDWHITEP, OUTPUT);
  digitalWrite(LEDBLUEP, HIGH);
  digitalWrite(LEDWHITEP, LOW);
  xTaskCreate(vTaskEncoders,    "Encoders",  2048, NULL, 3, NULL);
  xTaskCreate(vTaskWheels,      "Wheels",    4096, NULL, 3, NULL);
  xTaskCreate(vTaskPrintAndLED, "Print",     2048, NULL, 1, NULL);
}

void loop() {
  vTaskDelete(NULL);
  // test_odometry();
  // test_motor_simple_begin();
  // test_display_motor_speed();
 }
