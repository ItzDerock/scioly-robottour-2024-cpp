#include "pico/types.h"
class L298N {

public:
  L298N(uint enableFwd, uint enableRev, uint pwm);

  enum Direction { Backwards = 0, Forwards = 1 };

  /**
   * Spins the motor at the given power
   */
  void spin(Direction direction, float power);
  void spin(bool forwards, float power);

  /**
   * Stops.
   */
  void stop();

private:
  uint enableFwd, enableRev, pwm, pwmSlice, pwmChannel;
};
