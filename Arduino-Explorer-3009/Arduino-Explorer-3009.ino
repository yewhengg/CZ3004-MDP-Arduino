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
#define srSensorLeft1 A2    //PS3
#define srSensorLeft2 A3    //PS4
#define lrSensorRight1 A4   //PS5
// A5 = PS6

// Initialisation
DualVNH5019MotorShield md;
SharpIR SRSensorFront1(SharpIR:: GP2Y0A21YK0F, srSensorFront1);
SharpIR SRSensorFront2(SharpIR:: GP2Y0A21YK0F, srSensorFront2);
SharpIR SRSensorLeft1(SharpIR:: GP2Y0A21YK0F, srSensorLeft1);
SharpIR SRSensorLeft2(SharpIR:: GP2Y0A21YK0F, srSensorLeft2);
SharpIR LRSensorRight1(SharpIR:: GP2Y0A02YK0F, lrSensorRight1);

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
double srSensorFront1Distance = 0;
double srSensorFront2Distance = 0;
double srSensorLeft1Distance = 0;
double srSensorLeft2Distance = 0;
double lrSensorRight1Distance = 0;
RunningMedian srSensorFront1DistanceRM = RunningMedian(10);
RunningMedian srSensorFront2DistanceRM = RunningMedian(10);
RunningMedian srSensorLeft1DistanceRM = RunningMedian(10);
RunningMedian srSensorLeft2DistanceRM = RunningMedian(10);
RunningMedian lrSensorRight1DistanceRM = RunningMedian(10);
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
  pinMode(srSensorFront1, INPUT);
  pinMode(srSensorFront2, INPUT);
  pinMode(srSensorLeft1, INPUT);
  pinMode(srSensorLeft2, INPUT);
  pinMode(lrSensorRight1, INPUT);
  PCintPort::attachInterrupt(motorEncoderLeft1, leftEncoderInc, RISING);
  PCintPort::attachInterrupt(motorEncoderRight1, rightEncoderInc, RISING);
}

void loop()
{
  Serial.println("===== Testing Sensors =====");
  Serial.println("===== Average Method (Sample Size 10) =====");
  getSensorsDistance(10);
  Serial.println("===== Median Method =====");
  getSensorsDistanceRM(10);
  delay(1000); // 1 second delay
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

void getSensorsDistance(int n)
{
  Serial.print("|");
  Serial.print(getSRSensorFront1Distance(n));
  Serial.print("|");
  Serial.print(getSRSensorFront2Distance(n));
  Serial.print("|");
  Serial.print(getSRSensorLeft1Distance(n));
  Serial.print("|");
  Serial.print(getSRSensorLeft2Distance(n));
  Serial.print("|");
  Serial.print(getLRSensorRight1Distance(n));
  Serial.println("|");
}

void getSensorsDistanceRM(int n)
{
  Serial.print("|");
  Serial.print(getSRSensorFront1DistanceRM(n));
  Serial.print("|");
  Serial.print(getSRSensorFront2DistanceRM(n));
  Serial.print("|");
  Serial.print(getSRSensorLeft1DistanceRM(n));
  Serial.print("|");
  Serial.print(getSRSensorLeft2DistanceRM(n));
  Serial.print("|");
  Serial.print(getLRSensorRight1DistanceRM(n));
  Serial.println("|");
}

// ==================== getSRSensorFront1Distance() ====================
double getSRSensorFront1Distance(int n)
{
  // Average Method
  counter = 0;
  srSensorFront1Distance = 0;
  for (counter = 0; counter < n; counter++) {
    srSensorFront1Distance += srSensorFront1.getDistance();
  }
  srSensorFront1Distance = srSensorFront1Distance / n;
  return srSensorFront1Distance;
}

float getSRSensorFront1DistanceRM(int n)
{
  // Median Method
  counter = 0;
  srSensorFront1DistanceRM.clear();
  for (counter = 0; counter < n; counter++) {
    srSensorFront1DistanceRM.add(srSensorFront1.getDistance());
  }
  return srSensorFront1DistanceRM.getMedian();
}

// ==================== getSRSensorFront2Distance() ====================
double getSRSensorFront2Distance(int n)
{
  // Average Method
  counter = 0;
  srSensorFront2Distance = 0;
  for (counter = 0; counter < n; counter++) {
    srSensorFront2Distance += srSensorFront2.getDistance();
  }
  srSensorFront2Distance = srSensorFront2Distance / n;
  return srSensorFront2Distance;
}

float getSRSensorFront2DistanceRM(int n)
{
  // Median Method
  counter = 0;
  srSensorFront2DistanceRM.clear();
  for (counter = 0; counter < n; counter++) {
    srSensorFront2DistanceRM.add(srSensorFront2.getDistance());
  }
  return srSensorFront2DistanceRM.getMedian();
}

// ==================== getSRSensorLeft1Distance() ====================
double getSRSensorLeft1Distance(int n)
{
  // Average Method
  counter = 0;
  srSensorLeft1Distance = 0;
  for (counter = 0; counter < n; counter++) {
    srSensorLeft1Distance += srSensorLeft1.getDistance();
  }
  srSensorLeft1Distance = srSensorLeft1Distance / n;
  return srSensorLeft1Distance;
}

float getSRSensorLeft1DistanceRM(int n)
{
  // Median Method
  counter = 0;
  srSensorLeft1DistanceRM.clear();
  for (counter = 0; counter < n; counter++) {
    srSensorLeft1DistanceRM.add(srSensorLeft1.getDistance());
  }
  return srSensorLeft1DistanceRM.getMedian();
}

// ==================== getSRSensorLeft2Distance() ====================
double getSRSensorLeft2Distance(int n)
{
  // Average Method
  counter = 0;
  srSensorLeft2Distance = 0;
  for (counter = 0; counter < n; counter++) {
    srSensorLeft2Distance += srSensorLeft2.getDistance();
  }
  srSensorLeft2Distance = srSensorLeft2Distance / n;
  return srSensorLeft2Distance;
}

float getSRSensorLeft2DistanceRM(int n)
{
  // Median Method
  counter = 0;
  srSensorLeft2DistanceRM.clear();
  for (counter = 0; counter < n; counter++) {
    srSensorLeft2DistanceRM.add(srSensorLeft2.getDistance());
  }
  return srSensorLeft2DistanceRM.getMedian();
}

// ==================== getLRSensorRight1Distance() ====================
double getLRSensorRight1Distance(int n)
{
  // Average Method
  counter = 0;
  lrSensorRight1Distance = 0;
  for (counter = 0; counter < n; counter++) {
    lrSensorRight1Distance += lrSensorRight1.getDistance();
  }
  lrSensorRight1Distance = lrSensorRight1Distance / n;
  return lrSensorRight1Distance;
}

float getLRSensorRight1DistanceRM(int n)
{
  // Median Method
  counter = 0;
  lrSensorRight1DistanceRM.clear();
  for (counter = 0; counter < n; counter++) {
    lrSensorRight1DistanceRM.add(lrSensorRight1.getDistance());
  }
  return lrSensorRight1DistanceRM.getMedian();
}
