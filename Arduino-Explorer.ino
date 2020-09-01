https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/blob/master/Arduino/Integrated_code_final/Integrated_code_final.ino
https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino

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

DualVNH5019MotorShield md;
ZSharpIR SharpIR(IR, sharp_sensor_model);

// ===== setup() =====
void setup()
{
  Serial.begin(115200);
  md.init();
  pinMode(left_encoder_e1a, INPUT);
  pinMode(left_encoder_e1b, INPUT);
  pinMode(right_encoder_e2a, INPUT);
  pinMode(right_encoder_e2b, INPUT);
  PCintPort::attachInterrupt(left_encoder_e1a, leftEncoderInc, RISING);
  PCintPort::attachInterrupt(left_encoder_e1b, leftEncoderInc, RISING);
  PCintPort::attachInterrupt(right_encoder_e2a, rightEncoderInc, RISING);
  PCintPort::attachInterrupt(right_encoder_e2b, rightEncoderInc, RISING);
}

// ===== loop() =====
void loop()
{
  md.setSpeeds(300, 300);
  // Serial.println(SharpIR.distance());
  double duration_left_e1a = pulseIn(left_encoder_e1a, HIGH);
  double duration_left_e1b = pulseIn(left_encoder_e1b, HIGH);
  double duration_right_e2a = pulseIn(right_encoder_e2a, HIGH);
  double duration_right_e2b = pulseIn(right_encoder_e2b, HIGH);
  Serial.print("Left 1A: ");
  Serial.println(duration_left_e1a);
  Serial.print("Left 1B: ");
  Serial.println(duration_left_e1b);
  Serial.print("Right 2A: ");
  Serial.println(duration_right_e2a);
  Serial.print("Right 2B: ");
  Serial.println(duration_right_e2b);

  float rpm = leftEncoderValue / 
  long ST = millis();
  getRPM(ST);
}

void leftEncoderInc(void) {
  leftEncoderValue++;

  if (leftEncoderValue == 1) {

  }
  
}

void rightEncoderInc(void) {
  rightEncoderValue++;
}

void countPulse_MR() {
  counter_MR++;
  if(counter_MR == 1) startTime_MR = micros();
  else if(counter_MR == 11) {
    endTime_MR = micros();
    duration_MR = (endTime_MR - startTime_MR) / 10.0;
    input_MR = calculateRPM(duration_MR);
    counter_MR = 0;
  }
}


double calculateRPM(double pulse) {
  if (pulse == 0) return 0;
  else return 60.00 / (pulse *  562.25 / 1000000.00);
}
  
