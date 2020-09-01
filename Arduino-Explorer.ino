// https://github.com/nigelawj/cz3004
// https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/blob/master/Arduino/Integrated_code_final/Integrated_code_final.ino
// https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino

// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"

// Sensor Library
// https://github.com/guillaume-rico/SharpIR
#include "ZSharpIR.h"

#include "PinChangeInt.h"

// Definition of Parameters
#define IR A1
#define sharp_sensor_model 20150

#define left_encoder_e1a 3
#define left_encoder_e1b 5
#define right_encoder_e2a 11
#define right_encoder_e2b 13

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
ZSharpIR SharpIR(IR, sharp_sensor_model);

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
  md.setSpeeds(400, 400);
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
}
  
