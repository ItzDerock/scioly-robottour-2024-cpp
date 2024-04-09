#include "chassis.h"
#include "components/Motor.h"
#include "config.h"
#include "imu.h"
#include "utils.h"

#include "hardware/pio.h"
#include "hardware/structs/pio.h"
#include "pico/mutex.h"
#include "quadrature_encoder.pio.h"

#include <atomic>
#include <cmath>
#include <cstdio>

MotorController chassis::driveLeftController =
    MotorController(1.5, 0, 1.25, 15, 1.7, 0);

MotorController chassis::driveRightController =
    MotorController(1.5, 0, 1.25, 15, 1.7, 0);

mutex_t *odometryLock = new mutex_t();

#define ODOM_DEBUG false

LRT resetValues = {0, 0, 0};
LRT prevSensors = {0, 0, 0};

Position *state = new Position({0, 0, 0});
LRT *chassis::velocity = new LRT({0, 0, 0});

/**
 * Normalizes the sensor data to account for factors such as
 * gear ratio, encoder ticks, etc.
 *
 * @note - marked inline to reduce overhead of function calls
 *       - marked static so only accessible in odom.cpp
 *
 * @param position - raw position
 */
inline static double readSensorData(pio_hw_t *pio) {
  // count time by 14-counts-per-revolution and the approximate 20.4:1 gear
  // ratio (for extra credit, the exact ratio is 244904:12000 or 30613:1500).
  int rawCount = quadrature_encoder_get_count(pio, 0);
  return (float)rawCount / 14 / (244984.0f / 12000) * 6.5f;
}

void chassis::doOdometryTick() {
  // lock mutex
  if (!mutex_enter_timeout_ms(odometryLock, 500)) {
    printf("WARN ! Tracking failed to aquire mutex after 500ms.");
    return;
  };

  // 1. Store the current encoder values
  double left = readSensorData(pio0);
  double right = readSensorData(pio1);

  // 2. Calculate delta values
  double dL = left - prevSensors.left;
  double dR = right - prevSensors.right;

  // 3. Update the previous values
  prevSensors.left = left;
  prevSensors.right = right;

  // Calculate motor velocities
  velocity->left = dL / 0.01f;
  velocity->right = dR / 0.01f;

  // 4. total change since last reset
  // auto deltaLr = left - resetValues.left;
  // auto deltaRr = right - resetValues.right;

  // 5. Calculate new orientation
  double newTheta = getHeading() * M_PI / 180.0f;
  newTheta -= resetValues.theta;
  // flip
  newTheta = 2.0f * M_PI - newTheta;
  // printf("raw: %f, reset thetha: %f\n", getHeading(), resetValues.theta * 180
  // / M_PI);
  newTheta = utils::angleSquish(newTheta);
  // if (newTheta < 2 * M_PI)
  //   newTheta += 2 * M_PI;

  // 6. Calculate change in orientation
  double dTheta = newTheta - state->theta;
  double d = (dL + dR) / 2;

  // 7. Update the state
  state->y += d * cos(state->theta + dTheta / 2);
  state->x += d * sin(state->theta + dTheta / 2);
  state->theta = newTheta;

  // unlock mutex
  mutex_exit(odometryLock);
}

void chassis::odometryTask() {
  while (true) {
    doOdometryTick();

    float leftSpeed =
        chassis::driveLeftController.update(chassis::velocity->left);
    float rightSpeed =
        chassis::driveRightController.update(chassis::velocity->right);

    chassis::move(leftSpeed, rightSpeed);

    // auto pos = chassis::getPosition(true);
    // printf("x: %f, y: %f, h: %f\n", pos.x, pos.y, pos.theta);

    sleep_ms(10);
  }
}

void chassis::setPose(const Position &newState) {
  mutex_enter_blocking(odometryLock);

  state->x = newState.x;
  state->y = newState.y;
  state->theta = newState.theta;

  mutex_exit(odometryLock);
}

// void odom::reset(odom::RobotPosition startState) {
//   // aquire mutex
//   mutex.take();

//   // stop task
//   bool taskRunning = odomTask != nullptr;
//   if (taskRunning) {
//     odomTask->remove();
//     odomTask = nullptr;
//   }

//   // reset encoders
//   for (auto motor : drive_left) {
//     CHECK_SUCCESS(motor->set_zero_position(0), "drive_left");
//   }

//   for (auto motor : drive_right) {
//     CHECK_SUCCESS(motor->set_zero_position(0), "drive_right");
//   }

//   CHECK_SUCCESS(odom_middle.sensor->reset(), "odom_middle");
//   CHECK_SUCCESS(inertial->reset(true), "odom_imu");

//   // reset state
//   // state = startState;
//   state->x = startState.x;
//   state->y = startState.y;
//   state->theta = startState.theta;
//   resetValues.theta = startState.theta;

//   // reset prevSensors
//   prevSensors = {0, 0, 0, 0};

//   // delay 10ms to let the sensors reset
//   pros::delay(10);

//   // restart task
//   if (taskRunning)
//     initalize();

//   // release mutex
//   mutex.give();
// }

// void odom::reset() { reset({0, 0, 0}); }

void chassis::initializeOdometry() {
  mutex_init(odometryLock);

  for (int i = 0; i < 50; i++) {
    getHeading();
    sleep_ms(10);
  }

  resetValues.theta = getHeading() * M_PI / 180;
}

Position chassis::getPosition(bool degrees, bool standardPos) {
  mutex_enter_blocking(odometryLock);

  // get the state
  Position returnState =
      degrees ? Position({state->x, state->y,
                          static_cast<float>(state->theta * (180 / M_PI))})
              : *state;

  mutex_exit(odometryLock);

  // bearing -> standard form
  if (standardPos) {
    returnState.theta = utils::angleSquish(M_PI_2 - returnState.theta);
  }

  return returnState;
}

void chassis::moveVelocity(int left, int right) {
  chassis::driveLeftController.setTargetVelocity(left);
  chassis::driveRightController.setTargetVelocity(right);
}

void chassis::move(int left, int right) {
  // get direction
  bool leftFwd = left > 0;
  bool rightFwd = right > 0;

  // now take abs
  left = std::abs(left);
  right = std::abs(right);

  // and convert from [0, 127] to [0, 100]
  float leftSpeed = (float)left / 127 * 100;
  float rightSpeed = (float)right / 127 * 100;

  // now move
  driveLeft.spin(leftFwd, leftSpeed > 100 ? 100 : leftSpeed);
  driveRight.spin(rightFwd, rightSpeed > 100 ? 100 : rightSpeed);
}
