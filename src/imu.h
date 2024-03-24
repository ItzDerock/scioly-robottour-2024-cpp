#pragma once

#include "BNO08x/Adafruit_BNO08x.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

struct euler_t {
  float yaw; float pitch;
  float roll;
};

extern euler_t ypr;

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr,
                       bool degrees = false);

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector,
                         euler_t* ypr, bool degrees = false);

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector,
                         euler_t* ypr, bool degrees = false);

float getHeading();
