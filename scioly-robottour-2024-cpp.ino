/**
 * @file scioly-robottour-2024-cpp.ino
 * @brief This is the main file for the 2024 Robot Tour event.
 * @author Derock X <derock@derock.dev>
 * @license MIT
 */

#include <Arduino.h>

#include "PID.hpp"
#include "chassis.hpp"

unsigned long lastTime = 0;

void setup() { Serial.begin(9600); }

int i = 0;
void loop() {
  // in milliseconds
  double deltaTime = millis() - lastTime;
  lastTime = millis();

  // update odometry
  chassis::doOdometryUpdateTick();

  // move test
  chassis::move(255, 255);
  delay(1000);
  chassis::move(-255, -255);
  delay(1000);
  chassis::move(0, 0);

  delay(1000);
}
