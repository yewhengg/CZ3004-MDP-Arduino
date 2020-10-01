// References
// https://github.com/espressodepresso/CZ3004-MDP-Group-30-2020
// https://github.com/nigelawj/cz3004
// https://github.com/laksh22/MDP/tree/master/Arduino
// https://github.com/SoulXHades/CZ3004-MDP-Arduino
// https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/tree/master/Arduino
// https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino

// Robot Layout
//      <srSensorFront1>   <srSensorFront2>
//  <srSensorLeft1>            
//  <md1>                       <md2><lrSensorRight1>
//  <srSensorLeft2>            

// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"
// M1 = Right Motor, M2 = Left Motor

// Sensor Library
// https://github.com/qub1750ul/Arduino_SharpIR
//#include <SharpIR.h>
// https://github.com/guillaume-rico/SharpIR
#include <SharpIR.h>

// Pin Interrupt Library
// https://github.com/NicoHood/PinChangeInterrupt
#include "PinChangeInt.h"

// Running Median Library
// https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian
#include <RunningMedian.h>

// Parameters Definition
#define motorEncoderRight1 3
#define motorEncoderLeft1 11
#define srSensorFront1 A0   //PS1
#define srSensorFront2 A1   //PS2
#define srSensorFront3 A2   //PS3
#define srSensorLeft1 A3    //PS4
#define srSensorLeft2 A4    //PS5
#define lrSensorRight1 A5   //PS6
#define SRSensor_Model 1080
#define LRSensor_Model 20150

// Initialisation
DualVNH5019MotorShield md;

// SharpIR-Master Initialisation
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4928.71, 4.62709, -48.6083);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 16093.6, 14.4371, 157.702);
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 3840.37, 2.36529, -105.74);
SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 13483.7, 11.9145, 135.046);
SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 14166.9, 12.3137, 139.664);
SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 10237000, 1034.72, 9211.97);

// Parameters Declaration
// PID Calculation
unsigned long startTimeRightME = 0;
unsigned long endTimeRightME = 0;
unsigned long timeTakenRightME = 0;
unsigned long startTimeLeftME = 0;
unsigned long endTimeLeftME = 0;
unsigned long timeTakenLeftME = 0;
double RPMRight = 0;
double RPMLeft = 0;
int counterRight = 0;
int counterLeft = 0;
float kpLeftME = 2.70;
float kiLeftME = 0.1835;
float kdLeftME = 0.28;
float kpRightME = 2.10;
float kiRightME = 0.13;
float kdRightME = 0.20;
double k1LeftME = 0;
double k2LeftME = 0;
double k3LeftME = 0;
double k1RightME = 0;
double k2RightME = 0;
double k3RightME = 0;
double errorLeftME = 0;
double previousErrorLeftME = 0;
double previousPreviousErrorLeftME = 0;
double errorRightME = 0;
double previousErrorRightME = 0;
double previousPreviousErrorRightME = 0;
double PIDOutputLeftME = 0;
double previousPIDOutputLeftME = 0;
double PIDOutputRightME = 0;
double previousPIDOutputRightME = 0;
int setpoint = 80;
// Robot Movement
// Sensors
int counter = 0;
int sensorSampleSize = 50;
RunningMedian srSensorFront1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront3DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian lrSensorRight1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian analogReadings = RunningMedian(sensorSampleSize);
// Other Parameters
boolean startupFlag = true;
boolean flag = true;

// ==================== setup() ====================
void setup()
{
  Serial.begin(115200);
  md.init();
  pinMode(motorEncoderLeft1, INPUT);
  pinMode(motorEncoderRight1, INPUT);
  PCintPort::attachInterrupt(motorEncoderLeft1, leftEncoderInc, RISING);
  PCintPort::attachInterrupt(motorEncoderRight1, rightEncoderInc, RISING);
}

void loop()
{
  Serial.println("");
  //getSensorsDistanceRM(sensorSampleSize);
  getSensorsVoltageRM(sensorSampleSize);
  //Serial.print(round(SRSensorFront1.distance()/10));
  //Serial.print(" | ");
  //Serial.print(round(SRSensorFront2.distance()/10));
  //Serial.print(" | ");
  //Serial.print(round(SRSensorFront3.distance()/10));
  //Serial.print(" | ");
  //Serial.print(round(SRSensorLeft1.distance()/10));
  //Serial.print(" | ");
  //Serial.print(round(SRSensorLeft2.distance()/10));
  //Serial.print(" | ");
  //Serial.print(round(LRSensorRight1.distance()/10));
  Serial.println("");
  delay(1000);
}

// ==================== PID Calculation ====================

// ==================== leftEncoderInc() ====================
void leftEncoderInc()
{
  counterLeft++;
  if (counterLeft == 1) {
    startTimeLeftME = micros();
  } else if (counterLeft == 26) {
    endTimeLeftME = micros();
    timeTakenLeftME = (endTimeLeftME - startTimeLeftME) / 25;
    RPMLeft = calculateRPM(timeTakenLeftME);
    counterLeft = 0;
  }
}

// ==================== rightEncoderInc() ====================
void rightEncoderInc()
{
  counterRight++;
  if (counterRight == 1) {
    startTimeRightME = micros();
  } else if (counterRight == 26) {
    endTimeRightME = micros();
    timeTakenRightME = (endTimeRightME - startTimeRightME) / 25;
    RPMRight = calculateRPM(timeTakenRightME);
    counterRight = 0;
  }
}

// ==================== calculateRPM() ====================
double calculateRPM(unsigned long time)
{
  if (time == 0) {
    return 0;
  } else {
    return 60 / (time * 562.25 / 1000000);
  }
}

// ==================== PIDCalculation() ====================
void PIDCalculation(float kpLeftME, float kiLeftME, float kdLeftME, float kpRightME, float kiRightME, float kdRightME, int setpoint)
{
  // Calculate Digital PID Parameters
  k1LeftME = kpLeftME + kiLeftME + kdLeftME;
  k2LeftME = - kpLeftME - (2 * kdLeftME);
  k3LeftME = kdLeftME;
  k1RightME = kpRightME + kiRightME + kdRightME;
  k2RightME = - kpRightME - (2 * kdRightME);
  k3RightME = kdRightME;

  // Calculate Error
  errorLeftME = (setpoint - RPMLeft) / 130;
  errorRightME = (setpoint - RPMRight) / 130;

  // Calculate PID
  PIDOutputLeftME = previousPIDOutputLeftME + k1LeftME * errorLeftME + k2LeftME * previousErrorLeftME + k3LeftME * previousPreviousErrorLeftME;
  PIDOutputRightME = previousPIDOutputRightME + k1RightME * errorRightME + k2RightME * previousErrorRightME + k3RightME * previousPreviousErrorRightME;

  // Save PID and Error Values
  previousPIDOutputLeftME = PIDOutputLeftME;
  previousPIDOutputRightME = PIDOutputRightME;
  previousPreviousErrorLeftME = previousErrorLeftME;
  previousErrorLeftME = errorLeftME;
  previousPreviousErrorRightME = previousErrorRightME;
  previousErrorRightME = errorRightME;

  // Restart PID if Needed
  if (PIDOutputLeftME > 4 || PIDOutputRightME > 4) {
    restartPID();
  }
}

// ==================== restartPID() ====================
void restartPID() {
  RPMLeft = 0;
  RPMRight = 0;
  PIDOutputLeftME = 0;
  PIDOutputRightME = 0;
  previousPIDOutputLeftME = 0;
  previousPIDOutputRightME = 0;
  errorLeftME = 0;
  previousErrorLeftME = 0;
  previousPreviousErrorLeftME = 0;
  errorRightME = 0;
  previousErrorRightME = 0;
  previousPreviousErrorRightME = 0;
}

// ==================== Robot Movement ====================

//void goStraight1Grid()
//{
//}

//void turnLeft(int n)
//{  
//}

//void turnRight(int n)
//{
//}

// ==================== Sensors ====================

void getSensorsVoltageRM(int n)
{
  counter = 0;
  analogReadings.clear();
  for (counter = 0; counter < n; counter++) {
    analogReadings.add(analogRead(srSensorFront1));
    //analogReadings.add(analogRead(srSensorFront2));
    //analogReadings.add(analogRead(srSensorFront3));
    //analogReadings.add(analogRead(srSensorLeft1));
    //analogReadings.add(analogRead(srSensorLeft2));
    //analogReadings.add(analogRead(lrSensorRight1));
  }
  Serial.print("Median = ");
  Serial.println(analogReadings.getMedian());
  Serial.print("Average = ");
  Serial.println(analogReadings.getAverage(25));
}

void getSensorsDistanceRM(int n)
{
  counter = 0;
  srSensorFront1DistanceRM.clear();
  srSensorFront2DistanceRM.clear();
  srSensorFront3DistanceRM.clear();
  srSensorLeft1DistanceRM.clear();
  srSensorLeft2DistanceRM.clear();
  lrSensorRight1DistanceRM.clear();
  for (counter = 0; counter < n; counter++) {
    srSensorFront1DistanceRM.add(SRSensorFront1.distance());
    srSensorFront2DistanceRM.add(SRSensorFront2.distance());
    srSensorFront3DistanceRM.add(SRSensorFront3.distance());
    srSensorLeft1DistanceRM.add(SRSensorLeft1.distance());
    srSensorLeft2DistanceRM.add(SRSensorLeft2.distance());
    lrSensorRight1DistanceRM.add(LRSensorRight1.distance());
  }
  Serial.print("| ");
  Serial.print(srSensorFront1DistanceRM.getMedian());
  Serial.print(" | ");
  Serial.print(srSensorFront2DistanceRM.getMedian());
  Serial.print(" | ");
  Serial.print(srSensorFront3DistanceRM.getMedian());
  Serial.print(" | ");
  Serial.print(srSensorLeft1DistanceRM.getMedian());
  Serial.print(" | ");
  Serial.print(srSensorLeft2DistanceRM.getMedian());
  Serial.print(" | ");
  Serial.print(lrSensorRight1DistanceRM.getMedian());
  Serial.println(" |");
}
