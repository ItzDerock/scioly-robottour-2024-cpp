#include <cstdio>
#include <stdio.h>

#include <cmath>

#include "BNO08x/Adafruit_BNO08x.h"
#include "boards/pico.h"
#include "chassis.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

#include "config.h"
#include "imu.h"
#include "path.h"

L298N driveRight(6, 5, 4);
L298N driveLeft(7, 8, 9);

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
  multicore_launch_core1(chassis::odometryTask);

  // wait to get first imu

  // beep once
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(200);
  gpio_put(BEEPER_PIN, 0);

  // main loop
  while (true) {

    // gen points
    PathVector points = PathVector {
      Position{0, 0, 0},
      Position{0, 1, 0},
      Position{1, 1 ,0},
      Position{1, 2, 0},
      Position{3, 2, 0},
      Position{2, 2, 0},
      Position{2, 1, 0},
      Position{0, 0, 0},
      Position{0, 1, 0},
      Position{1, 1, 0},
      Position{1, 2, 0},
      Position{0, 2, 0},
      Position{0, 3, 0},
      Position{1, 3, 0}
    };

    // convert
    PathVector result;
    toAbsoluteCoordinates(points);
    interpolateAbsolutePath(points, result);

    // print
    for (int i = 0; i < result.size(); i++) {
      auto point = result.at(i);
      printf("%d: (%f, %f) at %f RPM\n", i, point.x, point.y, point.theta);
    }


    chassis::follow(result, 7, 50'000, true, false);
    // for (int i = -127; i <= 127; i++) {
    //   chassis::move(i, i);
    //   sleep_ms(100);
    //
    // }

    // sleep_ms(10'000);
    // int left = quadrature_encoder_get_count(pio0, 0);
    // int right = quadrature_encoder_get_count(pio1, 0);
    // float heading = getHeading();

    // printf("left,right: %d, %d, %f\n", left, right, heading);
    sleep_ms(10'000);
  }
}
