#pragma once

#include "BNO08x/Adafruit_BNO08x.h"
#define START_BUTTON_PIN 15
#define BEEPER_PIN 14
#define LEFT_WHEEL_ENCODER 10   // AB on 10,11
#define RIGHT_WHEEL_ENCODER 12  // AB on 12,13

Adafruit_BNO08x *imu = new Adafruit_BNO08x();
