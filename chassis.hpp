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

}  // namespace chassis