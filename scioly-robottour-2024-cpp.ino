/**
 * @file scioly-robottour-2024-cpp.ino
 * @brief This is the main file for the 2024 Robot Tour event.
 * @author Derock X <derock@derock.dev>
 * @license MIT
 */

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "PID.hpp"
#include "chassis.hpp"
#include "config.hpp"

SemaphoreHandle_t xSerialSemaphore = NULL;

void autonomous();

void setup() { 
  Serial.begin(9600);

  while (!Serial) { ; };
  if (xSerialSemaphore == NULL) {
    xSerialSemaphore = xSemaphoreCreateMutex();
    if (xSerialSemaphore != NULL)
      xSemaphoreGive(xSerialSemaphore);
  }

  xTaskCreate(
    chassis::taskOdometry,
    "Odometry", // human name
    256, // stack size (256 needed if logging enabled)
    NULL, // parameters
    2, // Priority (0-3)
    NULL // Task Handle
  );

  xTaskCreate(
    autonomous,
    "Main Robot Thread",
    256,
    NULL,
    2,
    NULL
  );
}

void autonomous() {
  while (true) {
    chassis::turnTo(90);
    vTaskDelay(134);
    chassis::turnTo(180);
    vTaskDelay(134); 
    chassis::turnTo(270); 
    vTaskDelay(134);
    chassis::turnTo(0);
    vTaskDelay(134);
  }
}

// FreeRTOS takes over, no loop.
void loop() {}
