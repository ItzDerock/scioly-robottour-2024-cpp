#include <atomic>
#include <cmath>
#include <cstdio>

#include "Adafruit_BNO08x.h"
#include "config.h"
#include "imu.h"
#include "sh2.h"

euler_t *ypr = new euler_t;
Adafruit_BNO08x *imu = new Adafruit_BNO08x(15);
std::atomic<CalibrationStatus> calibration_status =
    CALIBRATION_STATUS_UNCALIBRATED;

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

/**
 * Macro for checking the success of an operation. If the operation fails, the
 * function will log the error.
 */
#define CHECK_SUCCESS(operation)                                               \
  if (!operation) {                                                            \
    printf("[error] failed to " #operation "\n");                              \
    return;                                                                    \
  }

/**
 * To be called upon IMU reset. This function re-enables the required reports.
 */
void enableReports() {
  printf("[imu] Enabling reports.\n");
  CHECK_SUCCESS(imu->enableReport(SH2_ROTATION_VECTOR, 5'000));
  // CHECK_SUCCESS(imu->enableReport(SH2_MAGNETIC_FIELD_CALIBRATED));
  // CHECK_SUCCESS(imu->enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED));
}

// prevent realloc of this
sh2_SensorValue_t event;

float getHeading() {
  // Re-enable reports if the IMU was reset
  if (imu->wasReset()) {
    enableReports();
  }

  // read the sensor event
  if (!imu->getSensorEvent(&event)) {
    printf("[warn] getHeading() called, but no event waiting!\n");
    return ypr->yaw;
  };

  calibration_status.store((CalibrationStatus)event.status);

  switch (event.sensorId) {
  case SH2_ROTATION_VECTOR:
    quaternionToEulerRV(&event.un.rotationVector, ypr, true);
    break;
  default:
    break;
  }

  return ypr->yaw;
}

CalibrationStatus getCalibrationStatus() { return calibration_status.load(); }
