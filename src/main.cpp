#include <stdio.h>
#include <variant>

#include "Adafruit_BNO08x.h"
#include "chassis.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "path.h"
#include "pico/binary_info/code.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "quadrature_encoder.pio.h"

L298N driveRight(6, 5, 4);
L298N driveLeft(7, 8, 9);

const float FINISH_OFFSET = 16.f / 50;

// PathVector points = PathVector{
//     {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 2, 0},
//     {1, 2, 0}, {1, 3, 0}, {3, 3, 0}, {3, 2, 0}, {2 + FINISH_OFFSET, 2, 0}};
PathVector points = PathVector{
    {2, 0, 0}, {3, 1, 0}, {3, 3, 0}, {3, 0, 0}, {2, 1, 0},
    {1, 2, 0}, {0, 3, 0}, {0, 2, 0}, {1, 1, 0}, {2 + FINISH_OFFSET, 2, 0}};

// 0 | 1 | 2 | 3
#define START_QUAD 2
#define TARGET_SECONDS 40

// declared in button.cpp
int waitForMainButton(uint button_pin, uint led_pin);

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

  // initialize odometry tracking and set initial position
  chassis::initializeOdometry();

  const Position START_POSITION = {50 * START_QUAD + 25, -14, 0};
  chassis::setPose(START_POSITION);
  printf("[info] Odometry initializing at (%f, %f, %f)\n", START_POSITION.x,
         START_POSITION.y, START_POSITION.theta);

  multicore_launch_core1(chassis::odometryTask);

  // beep once
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(200);
  gpio_put(BEEPER_PIN, 0);

  // gen points

  // convert to centimeter
  printf("[info] running pathgen...\n");
  toAbsoluteCoordinates(points);

  printf("[debug] converted path:\n[debug]  <current %f, %f> ",
         chassis::getPosition().x, chassis::getPosition().y);
  for (Position segment : points) {
    printf("(%f, %f) -> ", segment.x, segment.y);
  }
  printf("\n");

  // interpolate all missing points
  std::vector<PathSegment> result;
  generatePath(points, result);
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
  int clicks = waitForMainButton(START_BUTTON_PIN, LIGHT_PIN);
  printf("[info] button clicked %d times\n", clicks);

  gpio_put(LIGHT_PIN, 0);
  chassis::setPose(START_POSITION, true);
  sleep_ms(100);

  // calculate end time
  int endTime = to_ms_since_boot(get_absolute_time()) + TARGET_SECONDS * 1000 +
                500; // over is better than under
  printf("[debug] current time is %d, target time is %d\n",
         to_ms_since_boot(get_absolute_time()), endTime);

  // run path
  // for (PathSegment segment : result) {
  for (int i = 0; i < result.size(); i++) {
    PathSegment segment = result.at(i);

    if (std::holds_alternative<float>(segment.data)) {
      float targetHeading = std::get<float>(segment.data);
      printf("turning to %d\n", targetHeading);
      chassis::turnTo(targetHeading);
    } else {
      // calculate remaining distance
      float remainingDistance = 0;

      for (int y = i + 1; y < result.size(); y++) {
        PathSegment remainingPart = result.at(y);

        if (std::holds_alternative<PathVector>(remainingPart.data)) {
          PathVector remainingPath = std::get<PathVector>(remainingPart.data);

          for (int z = 0; z < remainingPath.size() - 1; z++) {
            remainingDistance +=
                remainingPath[z].distance(remainingPath[z + 1]);
          }
        }
      }

      printf("Following path, remaining distance: %f\n", remainingDistance);
      chassis::follow(std::get<PathVector>(segment.data), 10, endTime,
                      remainingDistance);
    }
  }

  // once finished, pull both motors low
  driveLeft.stop();
  driveRight.stop();

  // done!
  while (true) {
    gpio_put(LIGHT_PIN, 1);
    sleep_ms(1'000);
    gpio_put(LIGHT_PIN, 0);
    sleep_ms(1'000);
  }
}
