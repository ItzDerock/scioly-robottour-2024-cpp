#include <L298N.h>
#include <QuadratureEncoder.h>

#include "chassis.hpp"
#include "config.hpp"

// define the motors
Encoders leftEncoder(LEFT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_B);
L298N leftMotor(LEFT_MOTOR_ENABLE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2);
Encoders rightEncoder(RIGHT_MOTOR_ENCODER_A, RIGHT_MOTOR_ENCODER_B);
L298N rightMotor(RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2);

// odometry global variables
chassis::Position currentPosition;
struct {
  double left, right;
} previousEncoder;

// utility functions
double toRealDistance(int encoderTicks) {
  return (double)encoderTicks / ENCODER_MULT /
         MOTOR_GEARING *       // this gives us the number of rotations
         WHEEL_CIRCUMFERENCE;  // this gives us the distance traveled
}

void odom::doOdometryUpdateTick() {
  // calculate distance traveled by each wheel
  double leftDistance = toRealDistance(leftEncoder.getEncoderCount());
  double rightDistance = toRealDistance(rightEncoder.getEncoderCount());

  // calculate delta in encoder ticks
  double dEncLeft = leftDistance - previousEncoder.left;
  double dEncRight = rightDistance - previousEncoder.right;

  previousEncoder.left = leftDistance;
  previousEncoder.right = rightDistance;

  // calculate delta distance and delta theta
  double dDistance = (dEncLeft + dEncRight) / 2;
  double dTheta = (dEncRight - dEncLeft) / TRACK_WIDTH;

  // calculate absolute x,y change
  currentPosition.x += dDistance * cos(currentPosition.theta + dTheta / 2);
  currentPosition.y += dDistance * sin(currentPosition.theta + dTheta / 2);
  currentPosition.theta += dTheta;
}

chassis::Position chassis::getPosition() { return currentPosition; }