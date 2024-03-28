#pragma once

#include "position.h"
#include <memory>
#include <vector>

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

void move(int left, int right);

void follow(std::shared_ptr<std::vector<Position>> pathPoints, float lookahead,
            int timeout, bool forwards, bool async);

} // namespace chassis
