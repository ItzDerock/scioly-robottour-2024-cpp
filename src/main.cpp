#include <stdio.h>

#include <cmath>

#include "BNO08x/Adafruit_BNO08x.h"
#include "boards/pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

const uint START_BUTTON_PIN = 15;
const uint BEEPER_PIN = 14;
const uint LEFT_WHEEL_ENCODER = 10;   // AB on 10,11
const uint RIGHT_WHEEL_ENCODER = 12;  // AB on 12,13

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

struct euler_t {
  float yaw; float pitch;
  float roll;
} ypr;

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr,
                       bool degrees = false) {
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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector,
                         euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i,
                    rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector,
                         euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i,
                    rotational_vector->j, rotational_vector->k, ypr, degrees);
}

int main() {
  // picotool configuration
  bi_decl(bi_program_description(
      "Science Olympiad Robot Tour - derock@derock.dev"));

  stdio_init_all();

  // initialize GPIO
  gpio_init(BEEPER_PIN);
  gpio_set_dir(BEEPER_PIN, GPIO_OUT);

  // initialize PIOs
  pio_add_program(pio0, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio0, 0, LEFT_WHEEL_ENCODER, 0);

  pio_add_program(pio1, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio1, 0, RIGHT_WHEEL_ENCODER, 0);

  // beep once
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(50);
  gpio_put(BEEPER_PIN, 0);

  // setup BNO
  Adafruit_BNO08x* mpu = new Adafruit_BNO08x();
  mpu->begin_I2C(BNO08x_I2CADDR_DEFAULT, i2c0, 16, 17);
  mpu->enableReport(SH2_ARVR_STABILIZED_RV, 5'000);

  // beep once
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(200);
  gpio_put(BEEPER_PIN, 0);

  // main loop
  while (true) {
    int left = quadrature_encoder_get_count(pio0, 0);
    int right = quadrature_encoder_get_count(pio1, 0);

    if (mpu->wasReset()) {
      printf("was reset\n");
      mpu->enableReport(SH2_ARVR_STABILIZED_RV, 5'000);
    }

    sh2_SensorValue_t event;
    if (!mpu->getSensorEvent(&event)) {
      printf("no event\n");
      sleep_ms(50);
      continue;
    };

    switch (event.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&event.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&event.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    printf("left,right: %d, %d, %f\n", left, right, ypr.yaw + 180);
    sleep_ms(50);
  }
}
