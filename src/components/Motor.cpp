#include "Motor.h"
#include "../utils.h"
#include <cstdio>

MotorController::MotorController(float Kp, float Ki, float Kd, float Ks,
                                 float Kv, float Ka)
    : Kp(Kp), Ki(Ki), Kd(Kd), Ks(Ks), Kv(Kv), Ka(Ka){};

void MotorController::setTargetVelocity(float target) {
  this->target = target;
}

float MotorController::update(float actual) {
  // calculate the error
  float error = target - actual;

  // --- PID stuff
  integral += error;
  float derivative = error - previousError;

  float kPOutput = Kp * error;
  float kIOutput = Ki * integral;
  float kDOutput = Kd * derivative;
  float output = kPOutput + kIOutput + kDOutput;
  // ---
  
  // --- FF stuff
  float expected = Ks * utils::sgn(target) + Kv * target + Ka * derivative;
  // ---

  previousError = error;

  // combine FF and PID
  return output + expected;
}
