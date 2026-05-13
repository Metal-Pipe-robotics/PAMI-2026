#include <Arduino.h>
#include <Servo.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_PWM_PIN  13

// ---------------------------------------------------------------------------
// Motion parameters
// ---------------------------------------------------------------------------
#define SERVO_ANGLE_MIN      35      // degrees — one end position
#define SERVO_ANGLE_MAX      155    // degrees — other end position
#define SERVO_SPEED_DEG_S    145     // degrees per second (1–360 typical)
#define SERVO_DWELL_MS       800   // milliseconds to wait at each end

// Step size per scheduler tick (1 tick = portTICK_PERIOD_MS ms)
// Using 1-degree steps; delay between steps is adjusted for speed.
#define STEP_DEG             1      // degrees moved per step
// Milliseconds between each 1-degree step
#define STEP_PERIOD_MS       ((uint32_t)(1000.0f * STEP_DEG / SERVO_SPEED_DEG_S))

static Servo g_servo;

static void servo_move_to(int &currentAngle, int targetAngle)
{
    int direction = (targetAngle > currentAngle) ? STEP_DEG : -STEP_DEG;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xStepPeriod = pdMS_TO_TICKS(STEP_PERIOD_MS);

    while (currentAngle != targetAngle)
    {
        currentAngle += direction;

        // Clamp to target to avoid overshoot on large step sizes
        if (direction > 0 && currentAngle > targetAngle) currentAngle = targetAngle;
        if (direction < 0 && currentAngle < targetAngle) currentAngle = targetAngle;

        g_servo.write(currentAngle);

        // Precise, drift-free delay
        vTaskDelayUntil(&xLastWakeTime, xStepPeriod);
    }
}

void vServoTask(void *pvParameters)
{
    (void)pvParameters;   // unused

    // Attach servo on the PWM pin
    g_servo.attach(SERVO_PWM_PIN);

    int currentAngle = SERVO_ANGLE_MIN;
    g_servo.write(currentAngle);

    // Brief settle time after attach
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;)
    {
        // --- Move to MAX end ---
        servo_move_to(currentAngle, SERVO_ANGLE_MAX);

        // Dwell at MAX
        vTaskDelay(pdMS_TO_TICKS(SERVO_DWELL_MS));

        // --- Move to MIN end ---
        servo_move_to(currentAngle, SERVO_ANGLE_MIN);

        // Dwell at MIN
        vTaskDelay(pdMS_TO_TICKS(SERVO_DWELL_MS));
    }

    // Should never reach here; tidy up just in case
    g_servo.detach();
    vTaskDelete(NULL);
}

void servo_start()
{
    xTaskCreate(
        vServoTask,         // Task function
        "ServoTask",        // Name (debug)
        2048,               // Stack size (bytes) — increase if using Serial inside task
        NULL,               // Parameters
        1,                  // Priority (1 = low, higher = more urgent)
        NULL                // Task handle (not needed here)
    );

}


