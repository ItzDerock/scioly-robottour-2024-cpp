#pragma once

#define MOTOR_GEARING 100  // 1:100 gearing
#define ENCODER_MULT 14

#define LEFT_MOTOR_ENABLE 13
#define LEFT_MOTOR_IN1 12
#define LEFT_MOTOR_IN2 11
#define LEFT_MOTOR_ENCODER_A 2
#define LEFT_MOTOR_ENCODER_B 3

#define RIGHT_MOTOR_ENABLE 8
#define RIGHT_MOTOR_IN1 10
#define RIGHT_MOTOR_IN2 9
#define RIGHT_MOTOR_ENCODER_A 4
#define RIGHT_MOTOR_ENCODER_B 5

#define WHEEL_CIRCUMFERENCE 20.42  // cm, d=65mm
#define TRACK_WIDTH 11.3             // cm

#include <semphr.h>
extern SemaphoreHandle_t xSerialSemaphore; 
