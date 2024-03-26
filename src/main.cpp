#include <cstdio>
#include <stdio.h>

#include <cmath>

#include "BNO08x/Adafruit_BNO08x.h"
#include "boards/pico.h"
#include "chassis.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

#include "config.h"
#include "imu.h"

L298N right(6, 5, 4);
L298N left(7, 8, 9);

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

  // beep once gpio_put(BEEPER_PIN, 1);
  sleep_ms(50);
  gpio_put(BEEPER_PIN, 0);

  // setup BNO
  // hard reset
  while (!imu->begin_I2C(BNO08x_I2CADDR_DEFAULT, i2c0, 16, 17)) {
    sleep_ms(100);
  };

  imu->enableReport(SH2_ARVR_STABILIZED_RV, 5'000);

  chassis::initializeOdometry();

  // wait to get first imu

  // beep once
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(200);
  gpio_put(BEEPER_PIN, 0);

  // main loop
  while (true) {
    chassis::doOdometryTick();

    for (int i = 0; i < 100; i++) {
      left.spin(L298N::Forwards, i);
    }
    // auto pos = chassis::getPosition(true);
    // printf("x: %f, y: %f, h: %f\n", pos.x, pos.y, pos.theta);
    
    sleep_ms(10);
    // int left = quadrature_encoder_get_count(pio0, 0);
    // int right = quadrature_encoder_get_count(pio1, 0);
    // float heading = getHeading();

    // printf("left,right: %d, %d, %f\n", left, right, heading);
    // sleep_ms(50);
  }
}
