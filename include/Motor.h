/**
 * Manage output voltage for DC motors
 * Consists of a PID controller + feedforward
 *
 * Designed for multicore use:
 *  - 1 core sets output, other updates the PID/ff
 *
 * @author - Derock X (derock@derock.dev)
 * @license - MIT
 */

#pragma once

class MotorController {

public:
  MotorController(float Kp, float Ki, float Kd, float Ks = 0, float Kv = 0,
                  float Ka = 0);

  void setTargetVelocity(float target);
  float update(float actual);

private:
  // PID Constants
  float Kp, Ki, Kd;

  // Feedforward constants
  float Ks = 0, Kv = 0, Ka = 0;

  // PID/FF runtime stuff
  float target = 0;
  float previousError = 0;
  float integral;
};
