#include "chassis.h"
#include "components/ExitCondition.h"
#include "components/PID.h"
#include "pico/time.h"
#include "utils.h"
#include <algorithm>

PIDController angularPID(4, 0, 20);
ExitCondition angularLargeExit(4, 300);
ExitCondition angularSmallExit(1, 100);

/**
 * @brief Turns to a given angle
 */
void chassis::turnTo(float degrees) {
  degrees = utils::angleSquish(degrees - 90, false);

  // reset angular controllers/exit conditions
  angularPID.reset();
  angularLargeExit.reset();
  angularSmallExit.reset();

  while (!angularLargeExit.getExit() && !angularSmallExit.getExit()) {
      // calculate error in degrees
      // this is because degrees makes it easier to tune the PID
      // as errors are larger
      Position pose = getPosition(true);
      double error = utils::angleError(pose.theta, degrees);

      // calculate the output from the PID
      float power = angularPID.update(error);
      angularLargeExit.update(error);
      angularSmallExit.update(error);

      // constrain the output
      power = std::clamp(power, -127.f, 127.f);
      // printf("power: %f\n", power);
      chassis::moveVelocity(-power, power);

      sleep_ms(10); 
    }

  // stop the drivetrain
  move(0, 0);
}
