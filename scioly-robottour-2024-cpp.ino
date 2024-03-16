/**
 * @file scioly-robottour-2024-cpp.ino
 * @brief This is the main file for the 2024 Robot Tour event.
 * @author Derock X <derock@derock.dev>
 * @license MIT
 */

#include <Arduino.h>
// #include <FreeRTOS.h>
// #include <task.h>
// #include <Arduino_FreeRTOS.h>
// #include <semphr.h>

#include "PID.hpp"
#include "chassis.hpp"
#include "config.hpp"

void setup() { 
  Serial1.begin(115200);
  Serial1.println("hello world");

  Serial.begin(115200);
  while (!Serial && millis() < 10000UL);
  Serial.println("started");

  pinMode(15, INPUT_PULLUP);
}

void setup1() {
  Serial1.print("Setting up odom");
  pinMode(14, OUTPUT);
  chassis::setupOdometry();
}

void loop1() {
  chassis::doOdometryUpdateTick();
  delay(15);
}

void loop() {
  int value = digitalRead(15);
  if (value == LOW) {
    digitalWrite(14, HIGH);
    delay(100);
    digitalWrite(14, LOW);
    delay(500);

    chassis::moveDistance(10);

    chassis::turnTo(90);
    delay(134 * 15);
    chassis::turnTo(180);
    delay(134 * 15);
    chassis::turnTo(270); 
    delay(134 * 15);
    chassis::turnTo(0);
    delay(134 * 15);
  }
}
