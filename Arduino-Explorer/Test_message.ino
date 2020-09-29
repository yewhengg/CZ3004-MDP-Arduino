//// https://github.com/nigelawj/cz3004
//// https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/blob/master/Arduino/Integrated_code_final/Integrated_code_final.ino
//// https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino
//
//// Motor Library
//// https://github.com/pololu/dual-vnh5019-motor-shield
//#include "DualVNH5019MotorShield.h"
//
//// Sensor Library
//// https://github.com/guillaume-rico/SharpIR
//#include "ZSharpIR.h"
//#include <SharpIR.h>
//
//#include "PinChangeInt.h"
//
//// Definition of Parameters
//#define IR1 A1 // front left
//#define IR2 A2 // front right
//#define IR3 A3 // right Long-range
//#define IR4 A4 // right short-range
//#define IR5 A5 // left short-range1
//#define IR6 A6 // left short-range2
//
//#define short_range_model 20150
//#define long_range_model 
//#define left_encoder_e1a 3
//#define left_encoder_e1b 5
//#define right_encoder_e2a 11
//#define right_encoder_e2b 13
//
//double leftEncoderValue = 0;
//double rightEncoderValue = 0;
//double rpm_left;
//double rpm_right;
//double readSensorDistance(int sensor);
//unsigned long startTimeLeftEncoder;
//unsigned long endTimeLeftEncoder;
//unsigned long startTimeRightEncoder;
//unsigned long endTimeRightEncoder;
//unsigned long durationLeftEncoder;
//unsigned long durationRightEncoder;
//
//DualVNH5019MotorShield md;
//
//SharpIR sr1(SharpIR:: GP2Y0A02YK0F, IR1); // front-left
//SharpIR sr2(SharpIR:: GP2Y0A02YK0F, IR2); // front-right
//SharpIR sr3(SharpIR:: GP2Y0A02YK0F, IR3);
//SharpIR sr4(SharpIR:: GP2Y0A02YK0F, IR4);
//SharpIR sr5(SharpIR:: GP2Y0A02YK0F, IR5);
//SharpIR sr6(SharpIR:: GP2Y0A02YK0F, IR6);
//
//// ===== setup() =====
//void setup()
//{
//  Serial.begin(115200);
//  md.init();
//  pinMode(left_encoder_e1a, INPUT);
//  //pinMode(left_encoder_e1b, INPUT);
//  pinMode(right_encoder_e2a, INPUT);
//  //pinMode(right_encoder_e2b, INPUT);
//  PCintPort::attachInterrupt(left_encoder_e1a, leftEncoderInc, RISING);
//  //PCintPort::attachInterrupt(left_encoder_e1b, leftEncoderInc, RISING);
//  PCintPort::attachInterrupt(right_encoder_e2a, rightEncoderInc, RISING);
//  //PCintPort::attachInterrupt(right_encoder_e2b, rightEncoderInc, RISING);
//}
//
//// ===== loop() =====
//void loop()
//{
//  readSensorDistance(1);
////  md.setSpeeds(400, 400);
//  // Serial.println(SharpIR.distance())
//}
//
//double readSensorDistance(int sensor){
//    double sum = 0;
//    double ir_distance[5];
//
//    if (sensor == 1) {
//    return sr1.getDistance();
//    }
//    if (sensor == 2) {
//      return sr2.getDistance();
//    }
//    if (sensor == 3) {
//      return sr3.getDistance();
//    }
//  
//    if (sensor == 4) {
//      return sr4.getDistance();
//    }
//    if (sensor == 5) {
//      return sr5.getDistance();
//    }
//    if (sensor == 6) {
//      return sr6.getDistance();
//    }
//    
//}
//
//
//void leftEncoderInc(void) {
//  leftEncoderValue++;
//  // Serial.println(leftEncoderValue);
//  if (leftEncoderValue == 1) {
//    startTimeLeftEncoder = micros();
//  } else if (leftEncoderValue == 101) {
//    endTimeLeftEncoder = micros();
//    durationLeftEncoder = (endTimeLeftEncoder - startTimeLeftEncoder) / 100;
//    rpm_left = 60.00 / (durationLeftEncoder * 562.25 / 1000000);
//    Serial.print("Left RPM: ");
//    Serial.println(rpm_left);
//    leftEncoderValue = 0;
//  }
//}
//
//void rightEncoderInc(void) {
//  rightEncoderValue++;
//  // Serial.print("Right Encoder Value: ");
//  // Serial.println(rightEncoderValue);
//  if (rightEncoderValue == 1) {
//    startTimeRightEncoder = micros();
//    // Serial.print("Start Time Value: ");
//    // Serial.println(startTimeRightEncoder);
//  } else if (rightEncoderValue == 101) {
//    endTimeRightEncoder = micros();
//    // Serial.print("End Time Value: ");
//    // Serial.println(endTimeRightEncoder);
//    durationRightEncoder = (endTimeRightEncoder - startTimeRightEncoder) / 100;
//    // Serial.print("Duration Value: ");
//    // Serial.println(durationRightEncoder);
//    rpm_right = 60.00 / (durationRightEncoder * 562.25 / 1000000);
//    Serial.print("Right RPM: ");
//    Serial.println(rpm_right);
//    rightEncoderValue = 0;
//  }
//}
//  
