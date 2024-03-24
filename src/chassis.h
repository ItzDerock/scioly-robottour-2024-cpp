#pragma once

struct Position {
  float x,y,z;
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

Position getPosition(bool degrees = false, bool standardPos = false);

}
