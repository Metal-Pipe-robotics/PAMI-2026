#include <Wire.h>
#include <VL53L1X.h>
#include "pins.hpp"

static VL53L1X sensor;
#define VL53_INT_PIN 23
volatile bool dataReady = false;

void IRAM_ATTR vl53_isr() {
  dataReady = true;
}

void VL53L1X_setup() 
{
  pinMode(VL53L1X_XSHUT, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(VL53L1X_XSHUT),
    vl53_isr,
    FALLING   // VL53L1X pulls GPIO1 LOW when data ready
  );


  Wire.begin(SDA_PIN, SCL_PIN);  // SDA, SCL for ESP32

  Serial.println("Initializing VL53L0X...");
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);  // Long, Medium, or Short
  sensor.setMeasurementTimingBudget(50000); // 50ms
  sensor.startContinuous(50);
  Serial.println("VL53L0X initialized.");
}

static unsigned long last_print = 0;
int VL53L1X_task() 
{
  uint16_t distance = -1;
  if (dataReady) {
    dataReady = false;

    distance = sensor.read();

    // if (!sensor.timeoutOccurred()) {
    //   distance = sensor.read();
    //   Serial.print("Distance: ");
    //   Serial.print(distance);
    //   Serial.println(" mm");
    // }

  }
  return distance;
}

