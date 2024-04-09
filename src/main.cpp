#include <cstdio>
#include <stdio.h>

#include <cmath>
#include <variant>

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
  gpio_init(START_BUTTON_PIN);
  gpio_init(LIGHT_PIN);

  gpio_pull_up(START_BUTTON_PIN);
  gpio_set_dir(START_BUTTON_PIN, GPIO_IN);

  gpio_set_dir(BEEPER_PIN, GPIO_OUT);
  gpio_set_dir(LIGHT_PIN, GPIO_OUT);

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
  chassis::setPose({25, 0, 0});
  multicore_launch_core1(chassis::odometryTask);

  // wait to get first imu

  // beep once
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(200);
  gpio_put(BEEPER_PIN, 0);

  // gen points
  PathVector points = PathVector{Position{0, 0, 0}, Position{0, 1, 0},
                                 Position{1, 1, 0}, Position{0, 1, 0}};

  // convert to centimeters
  printf("[info] running pathgen...\n");
  toAbsoluteCoordinates(points);

  printf("[debug] converted path:\n[debug]  <%f, %f> ",
         chassis::getPosition().x, chassis::getPosition().y);
  for (Position segment : points) {
    printf("(%f, %f) -> ", segment.x, segment.y);
  }
  printf("\n");

  // interpolate all missing points
  std::vector<PathSegment> result;
  interpolateAbsolutePath(points, result);
  printf("[info] pathgen done\n");

  // debug information
  printf("[debug] Planned path:\n");
  for (PathSegment segment : result) {
    if (std::holds_alternative<float>(segment.data)) {
      printf("[debug] turn to %f\n", std::get<float>(segment.data));
    } else {
      PathVector path = std::get<PathVector>(segment.data);
      printf("[debug] follow path:\n");

      for (Position position : path) {
        printf("[debug]  - %f, %f\n", position.x, position.y);
      }
    }
  }

  // led and wait for start
  while (true) {
    gpio_put(LIGHT_PIN, 1);
    sleep_ms(50);
    gpio_put(LIGHT_PIN, 0);

    if (!gpio_get(START_BUTTON_PIN)) // pulled up
      break;

    sleep_ms(50);
  }
  gpio_put(LIGHT_PIN, 0);
  sleep_ms(100);

  // run path
  for (PathSegment segment : result) {
    if (std::holds_alternative<float>(segment.data)) {
      float targetHeading = std::get<float>(segment.data);
      printf("turning to %d\n", targetHeading);
      chassis::turnTo(targetHeading);
    } else {
      printf("Following path\n");
      chassis::follow(std::get<PathVector>(segment.data), 7, 10'000, true,
                      false);
    }
  }

  // main loop
  while (true) {
    gpio_put(LIGHT_PIN, 1);
    sleep_ms(1'000);
    gpio_put(LIGHT_PIN, 0);
    sleep_ms(1'000);
  }
}
