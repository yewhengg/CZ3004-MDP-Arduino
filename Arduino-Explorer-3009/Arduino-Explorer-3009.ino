// References
// https://github.com/espressodepresso/CZ3004-MDP-Group-30-2020
// https://github.com/nigelawj/cz3004
// https://github.com/laksh22/MDP/tree/master/Arduino
// https://github.com/SoulXHades/CZ3004-MDP-Arduino
// https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/tree/master/Arduino
// https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino

// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"
// M1 = Right Motor, M2 = Left Motor

// Sensor Library
// https://github.com/qub1750ul/Arduino_SharpIR
//#include <SharpIR.h>
// https://github.com/guillaume-rico/SharpIR
#include "SharpIR.h"

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
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4810.44, 7.886, -40.303); // original b = 9.269
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 11680.9, 14.005, 121.366); // original b = 14.505
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5482.45, 10.396, -19.069); // original b = 9.396
SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 6852.53, 9.900, 16.1893);
SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 8273.7, 11.014, 63.616);
SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 24845.1, 21.719, 212.088); // original was b = 20.719

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
float kpLeftME = 2.72; // 2.69
float kiLeftME = 0.2;
float kdLeftME = 0.1;
float kpRightME = 3.12;
float kiRightME = 0.2;
float kdRightME = 0.1;
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
long oneGridDistance = 10980; // 8200;
long turnGridDistance = 9800;
long totalDistance = 0;
long turnDistanceLimit = 0;
boolean turnFlag = false;
boolean leftAngleCaliFlag;
// Sensors
String sensorOutput = "";
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
int srSensorFront1DistanceOutput = 0;
int srSensorFront2DistanceOutput = 0;
int srSensorFront3DistanceOutput = 0;
int srSensorLeft1DistanceOutput = 0;
int srSensorLeft2DistanceOutput = 0;
int lrSensorRight1DistanceOutput = 0;
RunningMedian analogReadings = RunningMedian(sensorSampleSize);
RunningMedian analogReadings2 = RunningMedian(sensorSampleSize);
RunningMedian analogReadings3 = RunningMedian(sensorSampleSize);
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
  Serial.flush();
}

void loop()
{
//  leftDistanceCalibrate();
delay(1500);
  getSensorsVoltageRM(10);
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
}

// ==================== Modes ====================

void exploration()
{
  Serial.println("===== exploration() =====");
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag) {
    if (Serial.available() > 0) {
      leftAngleCaliFlag = false;
      cc = char(Serial.read());
      if (cc == "U") {
        goStraight1Grid();
        delay(100);
        leftDistanceCalibrate();
        delay(100);
        leftAngleCalibrate();
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "L") {
        restartPID();
        turnLeft90Degrees(0.25);
        delay(100);
        leftDistanceCalibrate();
        delay(100);
        leftAngleCalibrate();
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "R") {
        turnRight90Degrees(0.25);
        delay(100);
        leftDistanceCalibrate();
        delay(100);
        leftAngleCalibrate();
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "D") {
        turnLeft90Degrees(0.25);
        delay(100);
        turnLeft90Degrees(0.25);
        delay(100);
        leftDistanceCalibrate();
        delay(100);
        leftAngleCalibrate();
        delay(100);
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if(cc == "C"){
        leftDistanceCalibrate();
        delay(100);
        leftAngleCalibrate();
      }
      else if (cc == "Z") {
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "#") {
        explorationFlag = false;
      } else if (cc == "A") {
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
}

void restartPID()
{
  restartLeftPID();
  restartRightPID();
}

void restartLeftPID()
{
  PIDOutputLeftME = 0;
  previousPIDOutputLeftME = 0;
  previousPreviousErrorLeftME = 0;
  previousErrorLeftME = 0;
  errorLeftME = 0;
  RPMLeft = 0;
}

void restartRightPID()
{
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
      md.setBrakes(400, 400);
      break;
    } else {
      moveForward();
      //leftDistanceCalibrate();
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void moveForward()
{
  //Serial.println("===== moveForward() =====");
  PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
  md.setSpeeds(PIDOutputRightME * 150, PIDOutputLeftME * 150);
  delayMicroseconds(4500); //5000
}

void turnLeft90Degrees(double n)
{
  //Serial.println("===== turnLeft90Degrees() =====");
  totalDistance = 0;
  counter = 0;
  turnFlag= true;
  if (n == 0.25) {
      turnDistanceLimit = 16500; 
    } else {
      turnDistanceLimit = 16500 + (250 * (n / 0.25));
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
//    // md.setSpeeds(250, -250);
      md.setSpeeds(PIDOutputRightME * 200, -PIDOutputLeftME * 200);
      totalDistance = totalDistance + RPMLeft + RPMRight;
      delayMicroseconds(4300);
    }
  }
}

void rotateLeft() {
  long total_Dis = 0;
  long limit = 14285; // 13775
  while(true) {
    if (total_Dis >= limit) {
      total_Dis = 0;
      md.setBrakes(400, 400);
      break;
    }
    else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 80);
      md.setSpeeds(PIDOutputRightME * 150, -PIDOutputLeftME * 150);
      delayMicroseconds(5000);
      //Serial.print(input_MR); Serial.print(", R  ||  ");
      total_Dis = total_Dis + RPMLeft + RPMRight;
      //total_Dis = total_Dis + input_ML + input_MR;
    }
  }
}

void rotateRight() {
  long total_Dis = 0;
  long limit = 14285; // 13775
  while(true) {
    if (total_Dis >= limit) {
      total_Dis = 0;
      md.setBrakes(400, 400);
      break;
    }
    else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 80);
      md.setSpeeds(-PIDOutputRightME * 150, PIDOutputLeftME * 150);
      delayMicroseconds(5000);
      //Serial.print(input_MR); Serial.print(", R  ||  ");
      total_Dis = total_Dis + RPMLeft + RPMRight;
      //total_Dis = total_Dis + input_ML + input_MR;
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
      turnDistanceLimit = 16500; 
    } else {
      turnDistanceLimit = 16500 + (250 * (n / 0.25));
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
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 60);
//      md.setSpeeds(-250, 250);
      md.setSpeeds(-PIDOutputRightME * 150, PIDOutputLeftME * 150);
      totalDistance = totalDistance + RPMLeft + RPMRight;
      delayMicroseconds(4250);
    }
  }
}

// ==================== Sensors ====================

void getSensorsVoltageRM(int n)
{
  counter = 0;
  analogReadings.clear();
  analogReadings2.clear();
  analogReadings3.clear();
  for (counter = 0; counter < n; counter++) {
    analogReadings.add(analogRead(srSensorFront1));
    analogReadings2.add(analogRead(srSensorFront2));
    analogReadings3.add(analogRead(srSensorFront3));
//    analogReadings.add(analogRead(srSensorLeft1));
//    analogReadings2.add(analogRead(srSensorLeft2));
//    analogReadings.add(analogRead(srSensorLeft2));
//    analogReadings.add(analogRead(lrSensorRight1));
  }
  Serial.print("Median = ");
  Serial.println(analogReadings.getMedian());
  Serial.println("---");
  Serial.println(analogReadings2.getMedian());
  Serial.println("---");
  Serial.println(analogReadings3.getMedian());
  
  Serial.print("Average = ");
  Serial.println(analogReadings.getAverage(10));
  Serial.println("---");
  Serial.println(analogReadings2.getAverage(10));
  Serial.println("---");
  Serial.println(analogReadings3.getAverage(10));
}

void getSensorsDistanceRM(int n)
{
  sensorOutput = "r";
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
  // Short Range Sensor Front 1
  srSensorFront1Distance = srSensorFront1DistanceRM.getMedian();
  if (1 <= srSensorFront1Distance && srSensorFront1Distance <= 16) {
    srSensorFront1DistanceOutput = 1;
  } else if (17 <= srSensorFront1Distance && srSensorFront1Distance <= 27) {
    srSensorFront1DistanceOutput = 2;
  } else {
    srSensorFront1DistanceOutput = -1;
  }
  // Short Range Sensor Front 2
  srSensorFront2Distance = srSensorFront2DistanceRM.getMedian();
  if (1 <= srSensorFront2Distance && srSensorFront2Distance <= 16) {
    srSensorFront2DistanceOutput = 1;
  } else if (17 <= srSensorFront2Distance && srSensorFront2Distance <= 27) {
    srSensorFront2DistanceOutput = 2;
  } else {
    srSensorFront2DistanceOutput = -1;
  }
  // Short Range Sensor Front 3
  srSensorFront3Distance = srSensorFront3DistanceRM.getMedian();
  if (1 <= srSensorFront3Distance && srSensorFront3Distance <= 16) {
    srSensorFront3DistanceOutput = 1;
  } else if (17 <= srSensorFront3Distance && srSensorFront3Distance <= 27) {
    srSensorFront3DistanceOutput = 2;
  } else {
    srSensorFront3DistanceOutput = -1;
  }
  // Short Range Sensor Left 1
  srSensorLeft1Distance = srSensorLeft1DistanceRM.getMedian();
  if (1 <= srSensorLeft1Distance && srSensorLeft1Distance <= 16) {
    srSensorLeft1DistanceOutput = 1;
  } else if (17 <= srSensorLeft1Distance && srSensorLeft1Distance <= 27) {
    srSensorLeft1DistanceOutput = 2;
  } else {
    srSensorLeft1DistanceOutput = -1;
  }
  // Short Range Sensor Left 2
  srSensorLeft2Distance = srSensorLeft2DistanceRM.getMedian();
  if (1 <= srSensorLeft2Distance && srSensorLeft2Distance <= 16) {
    srSensorLeft2DistanceOutput = 1;
  } else if (17 <= srSensorLeft2Distance && srSensorLeft2Distance <= 27) {
    srSensorLeft2DistanceOutput = 2;
  } else {
    srSensorLeft2DistanceOutput = -1;
  }
  // Long Range Sensor Right 1
  lrSensorRight1Distance = lrSensorRight1DistanceRM.getMedian();
  if (1 <= lrSensorRight1Distance && lrSensorRight1Distance <= 16) {
    lrSensorRight1DistanceOutput = 1;
  } else if (17 <= lrSensorRight1Distance && lrSensorRight1Distance <= 27) {
    lrSensorRight1DistanceOutput = 2;
  } else {
    lrSensorRight1DistanceOutput = -1;
  }
  //sensorOutput = sensorOutput + srSensorFront1DistanceOutput + "|" + srSensorFront2DistanceOutput + "|" + srSensorFront3DistanceOutput + "|" + srSensorLeft1DistanceOutput + "|" + srSensorLeft2DistanceOutput + "|" + lrSensorRight1DistanceOutput;
  sensorOutput = sensorOutput + srSensorFront1Distance + "|" + srSensorFront2Distance + "|" + srSensorFront3Distance + "|" + srSensorLeft1Distance + "|" + srSensorLeft2Distance + "|" + lrSensorRight1Distance;
  Serial.println(sensorOutput);
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
        delay(750);
        goStraight1Grid();
        delay(750);
        rotateLeft();
        delay(750);
        goStraight1Grid();
        delay(750);
        goStraight1Grid();
        delay(750);
        goStraight1Grid();
        delay(750);
        goStraight1Grid();
        delay(750);
        goStraight1Grid();
        delay(750);
        rotateLeft();
        delay(750);
        goStraight1Grid();
        delay(750);
        goStraight1Grid();
        delay(750);
        rotateRight();
        delay(750);
        rightFacing = false;
      }
      if (leftFacing = true) {
        leftFacing = false;
      }
      getSensorsDistanceRM(sensorSampleSize);
      if (srSensorFront1Distance > 20 && srSensorFront2Distance > 20 && srSensorFront3Distance > 20) {
        goStraight1Grid();
        delay(750);
      } else if (lrSensorRight1Distance > 20) {
        rotateRight();
        delay(750);
        rightFacing = true;
      } else if (srSensorLeft1Distance > 20 && srSensorLeft2Distance > 20) {
        rotateLeft();
        delay(750);
        leftFacing = true;
      }
    }
  }
}


// ==================== Robot Calibrate Orientation ====================

void calibrateTurnRight(double n){
  double totalDis = 0;
  boolean flag = true;
  double distanceLimit = 162000 * n;
  Serial.print("distanceLimit: ");
  Serial.print(distanceLimit);
  while(1){
    if (totalDis >= distanceLimit){
      md.setBrakes(-375, 375);
      totalDis = 0;
      break;
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
  double distanceLimit = 162000 * n;
  Serial.print("distanceLimit: ");
  Serial.print(distanceLimit);
  while(1){
    if (totalDis >= distanceLimit){
      md.setBrakes(375, -375);
      totalDis = 0;
      break;
    }
    else {
      md.setSpeeds(75, -75);
      totalDis = totalDis + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

void leftAngleCalibrate(){
//  Serial.print("SR SensorLeft 1: ");
//  Serial.println(SRSensorLeft1.distance());
//  Serial.print("SR SensorLeft 2: ");
//  Serial.println(SRSensorLeft2.distance());
  if((SRSensorLeft1.distance() < 120 and SRSensorLeft2.distance() < 120)){
  double diff = SRSensorLeft1.distance() - SRSensorLeft2.distance(); // will be in mm
  Serial.print("Diff outside while: " );
  Serial.println(diff);
    if(diff > 1 and diff < 90)
    {
      while(diff > 4){ // some certain difference when it rotates,
        calibrateTurnLeft(0.005); // turn slowly everytime until the diff is <= 0.5
        diff = SRSensorLeft1.distance() - SRSensorLeft2.distance();
//        Serial.print("Diff: " );
//        Serial.println(diff);
        delay(5);
      }
    }
    else if(diff < -1 and diff > -90){
      while(diff < -5){ // turn slowly everytime until the diff is >= -0.5
        calibrateTurnRight(0.005);
        diff = SRSensorLeft1.distance() - SRSensorLeft2.distance();
//        Serial.print("Diff: " );
//        Serial.println(diff);
        delay(5);
      }
    }
  }
}

void leftDistanceCalibrate(){
  if((SRSensorLeft1.distance() <= 45 and SRSensorLeft2.distance() <= 45) or (SRSensorLeft1.distance() > 70 and SRSensorLeft2.distance() > 70)){
    if(SRSensorLeft1.distance() <= 45 and SRSensorLeft2.distance() <= 45){
      Serial.print("Left 1:");
      Serial.print(SRSensorLeft1.distance());
      Serial.print("  Left 2:");
      Serial.print(SRSensorLeft2.distance());
      turnLeft90Degrees(0.25);
      delay(100);
      Serial.print("Front average: ");
      Serial.print(SRSensorFront2.distance());
      Serial.print(SRSensorFront3.distance());
      while(SRSensorFront2.distance() < 55 and SRSensorFront3.distance() < 55){
        md.setSpeeds(-75,-75);
        delay(5);
      }
      delay(100);
      turnRight90Degrees(0.25);
      
    }
    else if((SRSensorLeft1.distance() >= 70 and SRSensorLeft1.distance() <= 120) and (SRSensorLeft2.distance() >= 70 and SRSensorLeft2.distance() <= 120)){
      turnLeft90Degrees(0.25);
      delay(100);
      while(SRSensorFront2.distance() > 65 and SRSensorFront3.distance() >65){
        md.setSpeeds(75,75);
        delay(10);
      }
      delay(100);
      turnRight90Degrees(0.25);
    }
  }
}
  
