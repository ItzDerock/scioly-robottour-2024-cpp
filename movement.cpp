#include <L298N.h>
// #include <Arduino_FreeRTOS.h>
// #include <semphr.h>

#include "PID.hpp"
#include "chassis.hpp"
#include "config.hpp"

#define SETTLE_TIME 500
#define MOVEMENT_DEBUG true

// not sure if its the motor or the L298N 
// but analogWrite with values less than 110 
// do not spin the motor.
#define MINIMUM_SPEED 110
#define MIN_MIN_THRESHOLD 30 

L298N leftMotor(LEFT_MOTOR_ENABLE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2);
L298N rightMotor(RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2);

PIDController drivePID(20, 0.1, 16);
PIDController turnPID(14, 0.1, 16);

inline float angleSquish(float angle, bool degrees = false) {
  float half = degrees ? 180 : M_PI;

  while (angle < 0) angle += 2 * half;
  return fmod(angle, 2 * half);
}

inline float angleError(float angle1, float angle2, bool degrees = false) {
  float full = degrees ? 360 : 2 * M_PI;
  float remainder = fmod((angle1 - angle2), full);
  remainder = angleSquish(remainder, degrees);

  if (remainder > full / 2) {
    return (full - remainder);
  }

  return -1 * remainder;
}

void chassis::move(int left, int right) {
  // determine direction
  bool leftForward = left > 0;
  bool rightForward = right > 0;

  // analogWrite ranges from 0 to 255
  unsigned short leftSpeed = constrain(abs(left), 0, 255);
  unsigned short rightSpeed = constrain(abs(right), 0, 255);

  // unfortuantely motor doesnt spin if <100
  if (leftSpeed < MINIMUM_SPEED && leftSpeed > MIN_MIN_THRESHOLD)
    leftSpeed = MINIMUM_SPEED + 15;
  if (rightSpeed < MINIMUM_SPEED && rightSpeed > MIN_MIN_THRESHOLD)
    rightSpeed = MINIMUM_SPEED + 15;

  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);

  if (left == 0) leftMotor.stop();
  else leftMotor.run(leftForward ? L298N::Direction::FORWARD
                                 : L298N::Direction::BACKWARD);
  
  if (right == 0) rightMotor.stop();
  else rightMotor.run(rightForward ? L298N::Direction::FORWARD
                                   : L298N::Direction::BACKWARD);
}

void chassis::drive(double speed, double turn) {
  double leftSpeed = speed + turn;
  double rightSpeed = speed - turn;

  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

/**
 * Returns the distance between two points
 */
double distance(chassis::Position a, chassis::Position b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void chassis::moveTo(Position targetPosition) {
  int8_t sign = 1;

  // to finish movement, we should be settled for SETTLED_TIME
  unsigned int settledAmount = 0;
  double dist = distance(getPosition(), targetPosition);
  double distanceError = 999;
  double angularError = 999;

  // find the target position's X and Y
  Position initialPosition = getPosition();
  // double targetX = initialPosition.x + dist * sin(initialPosition.theta);
  // double targetY = initialPosition.y + dist * cos(initialPosition.theta);
  // Position targetPosition = {targetX, targetY, initialPosition.theta};

  // loop until we are settled
  while (settledAmount < SETTLE_TIME) {
    // get the current position
    Position position = getPosition();

    // calculate the error
    distanceError = dist - distance(position, initialPosition);
    // angularError = angleError(position.angle(targetPosition), M_PI_2 - position.theta);
    angularError = angleError(M_PI_2 - position.theta, position.angle(targetPosition));
    angularError *= 180 / M_PI;
    // angularError = position.theta - targetPosition.theta;

#if MOVEMENT_DEBUG
    Serial.print("D Error: ");
    Serial.print(distanceError);
    Serial.print(" | A Error: ");
    Serial.println(angularError * 180 / M_PI);
#endif

    // if we are settled, increment settledAmount
    if (fabs(distanceError) < 0.5) settledAmount += 15;
    else settledAmount = 0;

    // calculate the output
    double output = drivePID.update(distanceError);
    double angularOutput = turnPID.update(angularError);

    double left = sign * output + angularError;
    double right = sign * output - angularError;

    chassis::move(left, right);

    // vTaskDelay(1); // 15ms
    delay(15);
  }

  // stop the motors
  chassis::move(0,0);
}

void chassis::turnTo(float degrees) {
  turnPID.reset();
  float error = 999;
  unsigned int settledTime = 0; 

  while (settledTime < SETTLE_TIME) {
    Position pose = getPosition();
    
    // recalculate error
    error = angleError(pose.theta * 180 / M_PI, degrees, true);

    if (fabs(error) < 5) settledTime += 15;
    else settledTime = 0;

    Serial.print("A Error: ");
    Serial.print(error);
    Serial.print(" (");
    Serial.print(pose.theta * 180 / M_PI);
    Serial.print(" / ");
    Serial.print(degrees);
    Serial.print(")");

    double power = turnPID.update(error);
    chassis::move(-power, power);
    Serial.print(" -> ");
    Serial.println(power);

  }

  chassis::move(0, 0);
}
