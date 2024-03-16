#include <L298N.h>
#include "pio_encoder.h"
// #include <Arduino_FreeRTOS.h>

#include "chassis.hpp"
#include "config.hpp"
#include "MPU9250.h"

#define ODOM_DEBUG true
#define Serial Serial1

MPU9250 mpu;
float startTheta = 0;

// define the motors
PioEncoder leftEncoder(LEFT_MOTOR_ENCODER_A);
PioEncoder rightEncoder(RIGHT_MOTOR_ENCODER_A);

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
  double leftDistance = toRealDistance(leftEncoder.getCount());
  double rightDistance = toRealDistance(rightEncoder.getCount());

  // if (!mpu.update()) {
  //   Serial.println("no update");
  // }

  // float(atan2(mpu.getMagY(), mpu.getMagX())) * RAD_TO_DEG
  // float newTheta = (180 + mpu.getYaw()) * M_PI / 180; // to rads
  // if (newTheta > 2 * M_PI) newTheta -= 2 * M_PI;
  

  // [-180, 180]
  // [0, 360]


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
  // double dTheta = newTheta - currentPosition.theta;
  double dTheta = (dEncRight - dEncLeft) / 2 * TRACK_WIDTH;

  // calculate absolute x,y change
  currentPosition.x += dDistance * cos(currentPosition.theta + dTheta / 2);
  currentPosition.y += dDistance * sin(currentPosition.theta + dTheta / 2);
  // currentPosition.theta = newTheta;
  currentPosition.theta += dTheta;

#if ODOM_DEBUG
  // logging
  // if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 5) == pdTRUE) {
    Serial.print("X: ");
    Serial.print(currentPosition.x);
    Serial.print(" Y: ");
    Serial.print(currentPosition.y);
    Serial.print(" Theta: ");
    Serial.println(currentPosition.theta * 180 / M_PI);

   // xSemaphoreGive(xSerialSemaphore);
  //}
#endif
}

void chassis::setupOdometry() {
  digitalWrite(14, HIGH);
  delay(10);
  digitalWrite(14, LOW);

  // start encoders
  leftEncoder.begin();
  rightEncoder.begin();

  // start odom
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Serial1.println("pins set");
  Wire1.begin();
  Serial1.println("comms start");
  Wire1.setClock(400000);
  mpu.setup(0x68, {}, Wire1);
  Serial1.println("imu OK!");
  mpu.calibrateMag();
  mpu.calibrateAccelGyro();
  Serial1.println("calibrated");
  
  digitalWrite(14, HIGH);
  delay(50);
  digitalWrite(14, LOW);
  delay(50);

  int endTime = millis() + 5000;
  while (millis() < endTime) {
    if (mpu.update()) {
      startTheta = mpu.getYaw();
      delay(15);
    }
  };

  // set initial theta
  startTheta = mpu.getYaw() * M_PI / 180;

  // beep buzzer
  for (int i = 0; i < 3; i++) {
    digitalWrite(14, HIGH);
    delay(50);
    digitalWrite(14, LOW);
    delay(50);
  }
}

void chassis::taskOdometry(void *params) {
  while (true) {
    chassis::doOdometryUpdateTick();
    // vTaskDelay(1); // 15ms (1 tick)
    delay(100);
  }
}

chassis::Position chassis::getPosition() { return currentPosition; }
