// https://github.com/nigelawj/cz3004
// https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/blob/master/Arduino/Integrated_code_final/Integrated_code_final.ino
// https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino

// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"

// Sensor Library
// https://github.com/guillaume-rico/SharpIR
// #include "ZSharpIR.h"
// https://github.com/
#include <SharpIR.h>

// Other Libraries
#include "PinChangeInt.h"

//			<SR_F_1>			<SR_F_2>
//	<SR_L_1>		Robot			<SR_R_1>
//	<ME_L_1>							<ME_R_1>
//	<SR_L_2>							<LR_R_1>

// Definition of Parameters

// Definition of Sensors (SR = Short Range, LR = Long Range, L = Left, R = Right)
#define SRSensorF1 A0   //PS1
#define SRSensorF2 A1   //PS2
#define SRSensorL1 A2   //PS3
#define SRSensorL2 A3   //PS4
#define SRSensorR1 A4   //PS5
#define LRSensorR1 A5   //PS6

// Definition of Motors (ME = Motor Encoder, L = Left, R = Right)
#define MotorEncoderL1 3
#define MotorEncoderL2 5
#define MotorEncoderR1 11
#define MotorEncoderR2 13

// Initialise Sensors
SharpIR SRSensorFront1(SharpIR:: GP2Y0A02YK0F, SRSensorF1);
SharpIR SRSensorFront2(SharpIR:: GP2Y0A21YK0F, SRSensorF2);
SharpIR SRSensorLeft1(SharpIR:: GP2Y0A21YK0F, SRSensorL1);
SharpIR SRSensorLeft2(SharpIR:: GP2Y0A02YK0F, SRSensorL2);
SharpIR SRSensorRight1(SharpIR:: GP2Y0A21YK0F, SRSensorR1);
SharpIR LRSensorRight1(SharpIR:: GP2Y0A21YK0F, LRSensorR1);

// #define sharp_sensor_model 20150

double leftEncoderValue = 0;
double rightEncoderValue = 0;
double rpm_left;
double rpm_right;
unsigned long startTimeLeftEncoder;
unsigned long endTimeLeftEncoder;
unsigned long startTimeRightEncoder;
unsigned long endTimeRightEncoder;
unsigned long durationLeftEncoder;
unsigned long durationRightEncoder;

DualVNH5019MotorShield md;
// ZSharpIR SharpIR(IR, sharp_sensor_model);

// ===== setup() =====
void setup()
{
  Serial.begin(115200);
  md.init();
  pinMode(left_encoder_e1a, INPUT);
  //pinMode(left_encoder_e1b, INPUT);
  pinMode(right_encoder_e2a, INPUT);
  //pinMode(right_encoder_e2b, INPUT);
  PCintPort::attachInterrupt(left_encoder_e1a, leftEncoderInc, RISING);
  //PCintPort::attachInterrupt(left_encoder_e1b, leftEncoderInc, RISING);
  PCintPort::attachInterrupt(right_encoder_e2a, rightEncoderInc, RISING);
  //PCintPort::attachInterrupt(right_encoder_e2b, rightEncoderInc, RISING);
}

// ===== loop() =====
void loop()
{
  // md.setSpeeds(400, 400);
  // Serial.println(SharpIR.distance())
}

void leftEncoderInc(void) {
  leftEncoderValue++;
  // Serial.println(leftEncoderValue);
  if (leftEncoderValue == 1) {
    startTimeLeftEncoder = micros();
  } else if (leftEncoderValue == 101) {
    endTimeLeftEncoder = micros();
    durationLeftEncoder = (endTimeLeftEncoder - startTimeLeftEncoder) / 100;
    rpm_left = 60.00 / (durationLeftEncoder * 562.25 / 1000000);
    Serial.print("Left RPM: ");
    Serial.println(rpm_left);
    leftEncoderValue = 0;
  }
}

void rightEncoderInc(void) {
  rightEncoderValue++;
  // Serial.print("Right Encoder Value: ");
  // Serial.println(rightEncoderValue);
  if (rightEncoderValue == 1) {
    startTimeRightEncoder = micros();
    // Serial.print("Start Time Value: ");
    // Serial.println(startTimeRightEncoder);
  } else if (rightEncoderValue == 101) {
    endTimeRightEncoder = micros();
    // Serial.print("End Time Value: ");
    // Serial.println(endTimeRightEncoder);
    durationRightEncoder = (endTimeRightEncoder - startTimeRightEncoder) / 100;
    // Serial.print("Duration Value: ");
    // Serial.println(durationRightEncoder);
    rpm_right = 60.00 / (durationRightEncoder * 562.25 / 1000000);
    Serial.print("Right RPM: ");
    Serial.println(rpm_right);
    rightEncoderValue = 0;
}

double sensor1Distance(void) {

  return SRSensorFront1.getDistance()

}

double sensor2Distance(void) {

  return SRSensorFront2.getDistance()

}

double sensor3Distance(void) {

  return SRSensorLeft1.getDistance()

}

double sensor4Distance(void) {

  return SRSensorLeft2.getDistance()

}

double sensor5Distance(void) {

  return SRSensorRight1.getDistance()

}

double sensor6Distance(void) {

  return LRSensorRight1.getDistance()

}

void sensorsDistance(void) {
  Serial.print(sensor1Distance)
  Serial.print(" | ")
  Serial.print(sensor2Distance)
  Serial.print(" | ")
  Serial.print(sensor3Distance)
  Serial.print(" | ")
  Serial.print(sensor4Distance)
  Serial.print(" | ")
  Serial.print(sensor5Distance)
  Serial.print(" | ")
  Serial.println(sensor6Distance)
}
