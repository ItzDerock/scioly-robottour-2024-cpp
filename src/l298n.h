#include "pico/types.h"
class L298N {

public:
  L298N(uint enableFwd, uint enableRev, uint pwm);

  enum Direction { Forwards, Backwards };

  /**
   * Spins the motor at the given power
   */
  void spin(Direction direction, float power = -1);

  /**
   * Stops.
   */
  void stop();

private:
  uint enableFwd, enableRev, pwm, pwmSlice, pwmChannel;
};
