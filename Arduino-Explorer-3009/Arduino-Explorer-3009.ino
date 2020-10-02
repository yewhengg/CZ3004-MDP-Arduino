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
//SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4515.65, 2.996686, -69.1807);
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 3970.01, 1.35131, -71.8837);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 2951.5, -10.0054, -54.791);
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
float kpLeftME = 2.775; // 2.7
float kiLeftME = 0.185; //0.1835; 
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
long oneGridDistance = 13500;
long totalDistance = 0;
long turnDistanceLimit = 0;
boolean turnFlag = false;
// Sensors
int counter = 0;
int sensorSampleSize = 25;
RunningMedian srSensorFront1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront3DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian lrSensorRight1DistanceRM = RunningMedian(sensorSampleSize);
double srSensorFront1Distance = 0;
double srSensorFront2Distance = 0;
double srSensorFront3Distance = 0;
double srSensorLeft1Distance = 0;
double srSensorLeft2Distance = 0;
double lrSensorRight1Distance = 0;
RunningMedian analogReadings = RunningMedian(sensorSampleSize);
// Other Parameters
String cc = "";
boolean explorationFlag = false;
boolean fastestPathFlag = false;

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
  //Serial.println("===== loop() =====");
  if (Serial.available() > 0) {
    cc = char(Serial.read());
    if (cc == "E") {
      explorationFlag = true;
      exploration();
    } else if (cc == "F") {
      fastestPathFlag = true;
      //fastestPath();
    }
  }
  //Serial.println("");
  //getSensorsVoltageRM(sensorSampleSize);
  //delay(2000);
  //getSensorsDistanceRM(sensorSampleSize);
}

// ==================== Modes ====================

void exploration()
{
  //Serial.println("===== exploration() =====");
  delay(100);
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag) {
    if (Serial.available() > 0) {
      cc = char(Serial.read());
      if (cc == "U") {
        //Serial.println("Move Forward 1 Time");
        goStraight1Grid();
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "L") {
        //Serial.println("Left Turn 1 Time");
        turnLeft90Degrees(0.25);
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "R") {
        //Serial.println("Right Turn 1 Time");
        turnRight90Degrees(0.25);
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "D") {
        //Serial.println("Half Turn 1 Time");
        turnLeft90Degrees(0.25);
        delay(100);
        turnLeft90Degrees(0.25);
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "Z") {
        //Serial.println("Read Sensors Values");
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "#") {
        //Serial.println("End Exploration");
        explorationFlag = false;
      } else if (cc == "A") {
        Serial.println("Avoid Obstacle");
        avoidObstacle();
      }
    }
  }
}

// ==================== fastestPath() ====================

// ==================== PID Calculation ====================

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

double calculateRPM(unsigned long time)
{
  if (time == 0) {
    return 0;
  } else {
    return 60 / (time * 562.25 / 1000000);
  }
}

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
//  if (PIDOutputLeftME > 4 || PIDOutputRightME > 4) {
//    restartPID();
//  }

  // Restart PID if Needed
  if (PIDOutputLeftME > 4) {
      restartLeftPID();
    }
  if (PIDOutputRightME > 4) {
    restartRightPID();
  }
}

void restartPID() {
  restartLeftPID();
  restartRightPID();
}

void restartLeftPID() {
  PIDOutputLeftME = 0;
  previousPIDOutputLeftME = 0;
  previousPreviousErrorLeftME = 0;
  previousErrorLeftME = 0;
  errorLeftME = 0;
  RPMLeft = 0;
}

void restartRightPID(){
  PIDOutputRightME = 0;
  previousPIDOutputRightME = 0;
  previousPreviousErrorRightME = 0;
  previousErrorRightME = 0;
  errorRightME = 0;
  RPMRight = 0;
}

// ==================== Robot Movement ====================

void goStraight1Grid()
{
  //Serial.println("===== goStraight1Grid() =====");
  totalDistance = 0;
  while (1) {
    if (totalDistance >= oneGridDistance) {
      md.setBrakes(375, 375);
      break;
    } else {
      //moveForward();
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(PIDOutputRightME * 250, PIDOutputLeftME * 250);
      totalDistance = totalDistance + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

void moveForward()
{
  //Serial.println("===== moveForward() =====");
  PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
  md.setSpeeds(PIDOutputRightME * 250, PIDOutputLeftME * 250);
  delayMicroseconds(6000);
}

void turnLeft90Degrees(double n)
{
  //Serial.println("===== turnLeft90Degrees() =====");
  totalDistance = 0;
  counter = 0;
  turnFlag= true;
  if (n == 0.25) {
      turnDistanceLimit = 16600; 
    } else {
      turnDistanceLimit = 16600 + (250 * (n / 0.25));
    }
  while (turnFlag) {
    if (totalDistance >= turnDistanceLimit) {
      md.setBrakes(375, -375);
      totalDistance = 0;
      counter++;
      if (counter == 4 * n) {
        turnFlag = false;
        break;
      }
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(250, -250);
      totalDistance = totalDistance + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

void turnRight90Degrees(double n)
{
  //Serial.println("===== turnRight90Degrees() =====");
  totalDistance = 0;
  counter = 0;
  turnFlag= true;
  if (n == 0.25) {
      turnDistanceLimit = 16600; 
    } else {
      turnDistanceLimit = 16600 + (250 * (n / 0.25));
    }
  while (turnFlag) {
    if (totalDistance >= turnDistanceLimit) {
      md.setBrakes(-375, 375);
      totalDistance = 0;
      counter++;
      if (counter == 4 * n) {
        turnFlag = false;
        break;
      }
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(-250, 250);
      totalDistance = totalDistance + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

// ==================== Sensors ====================

void getSensorsVoltageRM(int n)
{
  counter = 0;
  analogReadings.clear();
  for (counter = 0; counter < n; counter++) {
    //analogReadings.add(analogRead(srSensorFront1));
    analogReadings.add(analogRead(srSensorFront2));
    //analogReadings.add(analogRead(srSensorFront3));
    //analogReadings.add(analogRead(srSensorLeft1));
    //analogReadings.add(analogRead(srSensorLeft2));
    //analogReadings.add(analogRead(lrSensorRight1));
  }
  Serial.print("Median = ");
  Serial.println(analogReadings.getMedian());
  //Serial.print("Average = ");
  //Serial.println(analogReadings.getAverage(10));
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
    srSensorFront1DistanceRM.add(round(SRSensorFront1.distance()/10));
    srSensorFront2DistanceRM.add(round(SRSensorFront2.distance()/10));
    srSensorFront3DistanceRM.add(round(SRSensorFront3.distance()/10));
    srSensorLeft1DistanceRM.add(round(SRSensorLeft1.distance()/10));
    srSensorLeft2DistanceRM.add(round(SRSensorLeft2.distance()/10));
    lrSensorRight1DistanceRM.add(round(LRSensorRight1.distance()/10));
  }
  Serial.print("r");
  Serial.print(srSensorFront1DistanceRM.getMedian());
  srSensorFront1Distance = srSensorFront1DistanceRM.getMedian();
  Serial.print("|");
  Serial.print(srSensorFront2DistanceRM.getMedian());
  srSensorFront2Distance = srSensorFront2DistanceRM.getMedian();
  Serial.print("|");
  Serial.print(srSensorFront3DistanceRM.getMedian());
  srSensorFront3Distance = srSensorFront3DistanceRM.getMedian();
  Serial.print("|");
  Serial.print(srSensorLeft1DistanceRM.getMedian());
  srSensorLeft1Distance = srSensorLeft1DistanceRM.getMedian();
  Serial.print("|");
  Serial.print(srSensorLeft2DistanceRM.getMedian());
  srSensorLeft2Distance = srSensorLeft2DistanceRM.getMedian();
  Serial.print("|");
  Serial.println(lrSensorRight1DistanceRM.getMedian());
  lrSensorRight1Distance = lrSensorRight1DistanceRM.getMedian();
}

// ==================== Checklist ====================

void avoidObstacle()
{
  Serial.println("===== avoidObstacle() =====");
  totalDistance = 0;
  boolean rightFacing = false;
  boolean leftFacing = false;
  while (1) {
    if (totalDistance >= 135000) { // 10 Grids
      md.setBrakes(400, 400);
      break;
    } else {
      if (rightFacing == true) {
        goStraight1Grid();
        delay(500);
        goStraight1Grid();
        delay(500);
        turnLeft90Degrees(0.25);
        delay(500);
        goStraight1Grid();
        delay(500);
        goStraight1Grid();
        delay(500);
        goStraight1Grid();
        delay(500);
        goStraight1Grid();
        delay(500);
        goStraight1Grid();
        delay(500);
        turnLeft90Degrees(0.25);
        delay(500);
        goStraight1Grid();
        delay(500);
        goStraight1Grid();
        delay(500);
        turnRight90Degrees(0.25);
        delay(500);
        rightFacing = false;
      }
      if (leftFacing = true) {
        leftFacing = false;
      }
      getSensorsDistanceRM(sensorSampleSize);
      if (srSensorFront1Distance > 20 && srSensorFront2Distance > 20 && srSensorFront3Distance > 20) {
        goStraight1Grid();
        delay(500);
      } else if (lrSensorRight1Distance > 20) {
        turnRight90Degrees(0.25);
        delay(500);
        rightFacing = true;
      } else if (srSensorLeft1Distance > 20 && srSensorLeft2Distance > 20) {
        turnLeft90Degrees(0.25);
        delay(500);
        leftFacing = true;
      }
    }
  }
}

/*
// ==================== Robot Calibrate Orientation ====================

void frontWallCalibrate(){
  if(SRSensorFront1.distance() <= 35 and SRSensorFront3.distance() <= 35){
    double diff = SRSensorFront1.distance() - SRSensorFront3.distance();
  
    if(diff > 0.1 and diff < 9)
    {
      while(diff > 0.5){ // some certain difference when it rotates,
        calibrateTurnLeft(0.0002); // turn slowly everytime until the diff is <= 0.5
        diff = SRSensorFront1.distance() - SRSensorFront3.distance();
        delay(30);
      }
    }
    else if(diff < -0.1 and diff > -9){
      while(diff < -0.5){ // turn slowly everytime until the diff is >= -0.5
        calibrateTurnRight(0.0002);
        diff = SRSensorFront1.distance() - SRSensorFront3.distance();
        delay(30);
      }
    }
  }
}

void leftWallCalibrate(){
  double diff = SRSensorLeft1.distance() - SRSensorLeft2.distance(); // will be in cm

  if(diff > 0.1 and diff < 9)
  {
    while(diff > 0.5){ // some certain difference when it rotates,
      calibrateTurnLeft(0.0002); // turn slowly everytime until the diff is <= 0.5
      diff = SRSensorLeft1.distance() - SRSensorLeft2.distance();
      delay(30);
    }
  }
  else if(diff < -0.1 and diff > -9){
    while(diff < -0.5){ // turn slowly everytime until the diff is >= -0.5
      calibrateTurnRight(0.0002);
      diff = SRSensorLeft1.distance() - SRSensorLeft2.distance();
      delay(30);
    }
  }
}

void calibrateTurnRight(double n){
  double totalDis = 0;
  boolean flag = true;
  double distanceLimit = 16200 * n;
  
  while(1){
    if (totalDis >= distanceLimit){
      md.setBrakes(-375, 375);
      totalDis = 0;
    }
    else {
      md.setSpeeds(-75, 75);
      totalDis = totalDis + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

void calibrateTurnLeft(double n){
  double totalDis = 0;
  boolean flag = true;
  double distanceLimit = 16200 * n;
  
  while(1){
    if (totalDis >= distanceLimit){
      md.setBrakes(375, -375);
      totalDis = 0;
    }
    else {
      md.setSpeeds(75, -75);
      totalDis = totalDis + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}
*/
