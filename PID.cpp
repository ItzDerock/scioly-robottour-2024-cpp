#include "PID.hpp"

#include <Arduino.h>

PIDController::PIDController(double kP, double kI, double kD)
    : _kP(kP), _kI(kI), _kD(kD), debug(false) {}

PIDController::PIDController(double kP, double kI, double kD, bool debug)
    : _kP(kP), _kI(kI), _kD(kD), debug(debug) {}

double PIDController::update(double error) {
  _integral += error;
  double derivative = error - _previousError;

  double kPOutput = _kP * error;
  double kIOutput = _kI * _integral;
  double kDOutput = _kD * derivative;

  // if kI is nan, reset the integral
  if (isnan(kIOutput)) {
    _integral = 0;
    kIOutput = 0;
  }

  if (debug) {
    Serial.print("P: ");
    Serial.print(kPOutput);
    Serial.print(" I: ");
    Serial.print(kIOutput);
    Serial.print(" D: ");
    Serial.print(kDOutput);
    Serial.print(" Error: ");
    Serial.println(error);
  }

  double output = kPOutput + kIOutput + kDOutput;

  _previousError = error;
  return output;
}

void PIDController::reset() {
  _previousError = 0;
  _integral = 0;
}