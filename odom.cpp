#include <L298N.h>
#include <QuadratureEncoder.h>
#include <Arduino_FreeRTOS.h>

#include "chassis.hpp"
#include "config.hpp"

#define ODOM_DEBUG false

// define the motors
Encoders leftEncoder(LEFT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_B);
Encoders rightEncoder(RIGHT_MOTOR_ENCODER_A, RIGHT_MOTOR_ENCODER_B);

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

void chassis::doOdometryUpdateTick() {
  // calculate distance traveled by each wheel
  double leftDistance = toRealDistance(leftEncoder.getEncoderCount());
  double rightDistance = toRealDistance(rightEncoder.getEncoderCount());

#if ODOM_DEBUG
  Serial.print("Left: ");
  Serial.println(leftDistance);
  Serial.print("Right: ");
  Serial.println(rightDistance);
#endif

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

#if ODOM_DEBUG
  // logging
  // if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 5) == pdTRUE) {
    Serial.print("X: ");
    Serial.print(currentPosition.x);
    Serial.print(" Y: ");
    Serial.print(currentPosition.y);
    Serial.print(" Theta: ");
    Serial.println(currentPosition.theta);

   // xSemaphoreGive(xSerialSemaphore);
  //}
#endif
}

void chassis::taskOdometry() {
  while (true) {
    chassis::doOdometryUpdateTick();
    vTaskDelay(1); // 15ms (1 tick)
  }
}

chassis::Position chassis::getPosition() { return currentPosition; }