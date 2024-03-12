#include <L298N.h>

#include "PID.hpp"
#include "chassis.hpp"
#include "config.hpp"

L298N leftMotor(LEFT_MOTOR_ENABLE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2);
L298N rightMotor(RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2);

PIDController drivePID(0.5, 0.1, 0.1);
PIDController turnPID(0.5, 0.1, 0.1);

void chassis::move(int left, int right) {
  // determine direction
  bool leftForward = left > 0;
  bool rightForward = right > 0;

  // analogWrite ranges from 0 to 255
  unsigned short leftSpeed = constrain(abs(left), 0, 255);
  unsigned short rightSpeed = constrain(abs(right), 0, 255);

  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
  leftMotor.run(leftForward ? L298N::Direction::FORWARD
                            : L298N::Direction::BACKWARD);
  rightMotor.run(rightForward ? L298N::Direction::FORWARD
                              : L298N::Direction::BACKWARD);
}

void chassis::drive(double speed, double turn) {
  double leftSpeed = speed + turn;
  double rightSpeed = speed - turn;

  // fit into unsigned short (16 bytes)
  leftSpeed = constrain(leftSpeed, 0, 65536);
  rightSpeed = constrain(rightSpeed, 0, 65536);

  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}