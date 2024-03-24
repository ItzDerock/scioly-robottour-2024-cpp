#include "chassis.h"
#include "config.h"
#include "imu.h"

#include "hardware/pio.h"
#include "hardware/structs/pio.h"
#include "pico/lock_core.h"
#include "quadrature_encoder.pio.h"

#include <atomic>
#include <cmath>

auto_init_mutex(odometryLock);

#define ODOM_DEBUG false

struct {
  double left, right, theta;
} prevSensors = {0, 0, 0};

struct {
  double left, right, theta;
} resetValues = {0, 0, 0};

Position *state = new Position({0, 0, 0});

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
  return rawCount / 14 / (244984 / 12000); // TODO: wheel diam
}

void chassis::doOdometryTick() {
  // lock mutex
  odometryLock.take();

  // 1. Store the current encoder values
  double left = readSensorData(pio0);
  double right = readSensorData(pio1);

  // 2. Calculate delta values
  double dL = left - prevSensors.left;
  double dR = right - prevSensors.right;

  // 3. Update the previous values
  prevSensors.left = left;
  prevSensors.right = right;

  // 4. total change since last reset
  // auto deltaLr = left - resetValues.left;
  // auto deltaRr = right - resetValues.right;

  // 5. Calculate new orientation
  double newTheta = resetValues.theta + getHeading() * M_PI / 180;
  if (newTheta < 2 * M_PI)
    newTheta += 2 * M_PI;

  // 6. Calculate change in orientation
  double dTheta = newTheta - state->theta;
  double d = (dL + dR) / 2;

  // copy the state so we can calculate the velocity
  RobotPosition prevState = *state;

  // 7. Update the state
  state->y += d * cos(state->theta + dTheta / 2);
  state->x += d * sin(state->theta + dTheta / 2);
  state->theta = newTheta;

  // calculate the velocity
  // divide by 10 because this function is called every 10ms
  // multiply by 1000 to get it in inches per second
  velocity = (state->distance(prevState) / 10) * 1000;

  // The following is the legacy odometry algorithm
  // it has been replaced with the above code

  /*
    // 7. Calculate local offset for dTheta = 0
    RobotPosition localOffset = {0, 0, 0};

    if (dTheta == 0) {
      localOffset.x = dC;
      localOffset.y = dR;
    } else {
      // 8. Otherwise, calculate local offset with formula.
      localOffset.x = 2 * sin(dTheta / 2) * (dC / dTheta +
    (odom_middle.offset));

      localOffset.y = 2 * sin(dTheta / 2) * (dR / dTheta + (odom_right.offset));
    }

    // 9. Calculate the average orientation
    double thetam = state->theta + dTheta / 2;

    // 10. Calculate the global offset
    RobotPosition globalOffset = {0, 0, 0};

    // convert local offset to polar coordinates
    double r =
        sqrt(localOffset.x * localOffset.x + localOffset.y * localOffset.y);
    double theta = atan2(localOffset.y, localOffset.x);

    // subtract thetam from the angle component
    theta -= thetam;

    // convert back to Cartesian coordinates
    globalOffset.x = r * cos(theta);
    globalOffset.y = r * sin(theta);

    // 11. Update the global position
    state->x += globalOffset.x;
    state->y += globalOffset.y;
    state->theta = newTheta;
  */

#if ODOM_DEBUG
  logger::log(logger::Route::RobotPosition, {state->x, state->y, state->theta});

  // I wish there was a more elegant way to do this
  logger::log(logger::Route::RobotVelocity,
              {
                  drive_left_back->get_actual_velocity(),
                  (double)drive_left_back->get_target_velocity(),
                  drive_right_back->get_actual_velocity(),
                  (double)drive_right_back->get_target_velocity(),
              });
#endif

  // unlock mutex
  mutex.give();
}

void odom::updateLoop() {
  while (true) {
    update();
    pros::delay(10);
  }
}

void odom::initalize() {
  if (odomTask != nullptr) {
    std::cout << "WARNING: odom::init() called when odomTask is not null"
              << std::endl;
    return;
  }

  odomTask = new pros::Task(updateLoop);
}

// macro to handle errors properly
#define CHECK_SUCCESS(fn, name)                                                \
  if (fn != 1) {                                                               \
    std::cerr << "FAILED TO RESET ODOMETRY!" << std::endl                      \
              << "errorno: " << errno << std::endl                             \
              << "at: " << name << std::endl;                                  \
  }

void odom::reset(odom::RobotPosition startState) {
  // aquire mutex
  mutex.take();

  // stop task
  bool taskRunning = odomTask != nullptr;
  if (taskRunning) {
    odomTask->remove();
    odomTask = nullptr;
  }

  // reset encoders
  for (auto motor : drive_left) {
    CHECK_SUCCESS(motor->set_zero_position(0), "drive_left");
  }

  for (auto motor : drive_right) {
    CHECK_SUCCESS(motor->set_zero_position(0), "drive_right");
  }

  CHECK_SUCCESS(odom_middle.sensor->reset(), "odom_middle");
  CHECK_SUCCESS(inertial->reset(true), "odom_imu");

  // reset state
  // state = startState;
  state->x = startState.x;
  state->y = startState.y;
  state->theta = startState.theta;
  resetValues.theta = startState.theta;

  // reset prevSensors
  prevSensors = {0, 0, 0, 0};

  // delay 10ms to let the sensors reset
  pros::delay(10);

  // restart task
  if (taskRunning)
    initalize();

  // release mutex
  mutex.give();
}

void odom::reset() { reset({0, 0, 0}); }

odom::RobotPosition odom::getPosition(bool degrees, bool standardPos) {
  // take mutex to prevent reading while writing
  // technically not necessary as the cpu is single threaded
  // but it's good practice
  mutex.take();

  // get the state
  RobotPosition returnState =
      degrees ? RobotPosition(state->x, state->y, state->theta * (180 / M_PI))
              : *state;

  mutex.give();

  // bearing -> standard form
  if (standardPos) {
    returnState.theta = utils::angleSquish(M_PI_2 - returnState.theta);
  }

  return returnState;
}

float odom::getVelocity() {
  // variable is atomic so no need to lock mutex
  return velocity;
}
