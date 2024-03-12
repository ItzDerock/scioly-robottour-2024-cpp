#pragma once

namespace chassis {

struct Position {
  double x, y, theta;
};

/**
 * Updates the robot position
 */
void doOdometryUpdateTick();

/**
 * Returns the current robot position
 */
Position getPosition();

/**
 * Moves the robot
 */
void move(int left, int right);

void drive(double speed, double turn);

}  // namespace chassis