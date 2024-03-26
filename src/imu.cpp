#include <cmath>
#include <cstdio>

#include "BNO08x/Adafruit_BNO08x.h"
#include "config.h"
#include "imu.h"

euler_t *ypr = new euler_t;
Adafruit_BNO08x *imu = new Adafruit_BNO08x(15);

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr,
                       bool degrees) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  ypr->yaw = std::atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = std::asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = std::atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector,
                         euler_t *ypr, bool degrees) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i,
                    rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector,
                         euler_t *ypr, bool degrees) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i,
                    rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float getHeading() {
  if (imu->wasReset()) {
    printf("was reset\n");
    imu->enableReport(SH2_ARVR_STABILIZED_RV, 5'000);
  }

  sh2_SensorValue_t event;
  if (!imu->getSensorEvent(&event)) {
    printf("no event\n");
    return ypr->yaw;
  };

  switch (event.sensorId) {
  case SH2_ARVR_STABILIZED_RV:
    quaternionToEulerRV(&event.un.arvrStabilizedRV, ypr, true);
  case SH2_GYRO_INTEGRATED_RV:
    // faster (more noise?)
    quaternionToEulerGI(&event.un.gyroIntegratedRV, ypr, true);
    break;
  }

  return ypr->yaw;
}
