#pragma once

#include "Adafruit_BNO08x.h"
#include "l298n.h"

#define START_BUTTON_PIN 3
#define BEEPER_PIN 14
#define LIGHT_PIN 2
#define LEFT_WHEEL_ENCODER 10  // AB on 10,11
#define RIGHT_WHEEL_ENCODER 12 // AB on 12,13
#define DRIVE_TRACK_WIDTH 20   // cm

extern Adafruit_BNO08x *imu;
extern L298N driveLeft;
extern L298N driveRight;
