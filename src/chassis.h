#pragma once

struct Position {
  float x, y, theta;
};

namespace chassis {

/**
  * Handles one odometry tick
  */
void doOdometryTick();

/**
  * Runs odometry in a loop
  */
void odometryTask();

void initializeOdometry();

Position getPosition(bool degrees = false, bool standardPos = false);

}
