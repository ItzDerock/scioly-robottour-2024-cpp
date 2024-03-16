#pragma once

namespace chassis {

struct Position {
  double x, y, theta;

  float angle(chassis::Position other) const {
    return atan2(other.y - this->y, other.x - this->x);
  }
};

/**
 * Updates the robot position
 */
void doOdometryUpdateTick();
void setupOdometry();
void taskOdometry(void *params); // odom task

/**
 * Returns the current robot position
 */
Position getPosition();

/**
 * Moves the robot
 */
void move(int left, int right);
void drive(double speed, double turn);
void moveTo(Position targetPosition);
void turnTo(float degrees);

}  // namespace chassis
