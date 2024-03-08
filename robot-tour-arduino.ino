#include <L298N.h>
#include <L298NX2.h>
#include <QuadratureEncoder.h>

#include "PID.h"

#define MOTOR_GEARING 100  // 1:100 gearing
#define ENCODER_MULT 14

#define LEFT_MOTOR_ENABLE 13
#define LEFT_MOTOR_IN1 12
#define LEFT_MOTOR_IN2 11
#define LEFT_MOTOR_ENCODER_A 2
#define LEFT_MOTOR_ENCODER_B 3

#define RIGHT_MOTOR_ENABLE 10
#define RIGHT_MOTOR_IN1 9
#define RIGHT_MOTOR_IN2 8
#define RIGHT_MOTOR_ENCODER_A 4
#define RIGHT_MOTOR_ENCODER_B 5

unsigned long lastTime = 0;

Encoders leftEncoder(LEFT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_B);
double leftCurrentRPM = 0;
double leftPreviousTicks = 0;
PIDController leftPID(120, 0.0, 20);
L298N leftMotor(LEFT_MOTOR_ENABLE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2);

double targetRPM = 10.0;

void setup() {
  Serial.begin(9600);
  leftPID.reset();
}

int i = 0;
void loop() {
  double deltaTime = (millis() - lastTime) / 1000.0;

  // calculate the current RPM
  double currentTicks =
      leftEncoder.getEncoderCount() / ENCODER_MULT / MOTOR_GEARING;
  leftCurrentRPM = (currentTicks - leftPreviousTicks) / deltaTime * 60;
  // leftCurrentRPM = encoderRPM / MOTOR_GEARING / ENCODER_MULT;
  leftPreviousTicks = currentTicks;

  // calculate the new motor speed
  double leftError = targetRPM - leftCurrentRPM;
  double leftMotorSpeed = leftPID.update(leftError);

  // set the motor speed
  leftMotor.setSpeed(leftMotorSpeed);
  leftMotor.run(leftError > 0 ? L298N::Direction::FORWARD
                              : L298N::Direction::BACKWARD);

  // print RPM every 100ms
  if (++i >= 50) {
    Serial.print("RPM: ");
    Serial.println(leftCurrentRPM);

    Serial.print("Target speed: ");
    Serial.println(leftMotorSpeed);
    i = 0;
  };

  // if serial data is available, read it in
  if (Serial.available() > 0) {
    targetRPM = Serial.parseFloat();
    Serial.print("New target RPM: ");
    Serial.println(targetRPM);
  }

  delay(10);
}
