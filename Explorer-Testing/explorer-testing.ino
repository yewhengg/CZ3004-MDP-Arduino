#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "PinChangeInt.h"
#include <RunningMedian.h>

// Parameters Definition
#define motorEncoderRight1 3
#define motorEncoderLeft1 11
#define srSensorFront1 A0 // PS1
#define srSensorFront2 A1 // PS2
#define srSensorFront3 A2 // PS3
#define srSensorLeft1 A3  // PS4
#define srSensorLeft2 A4  // PS5
#define lrSensorRight1 A5 // PS6
#define SRSensor_Model 1080
#define LRSensor_Model 20150
// Tuning parameters for week 9

// PID
float kpLeftME = 2.70;
float kiLeftME = 0.19;
float kdLeftME = 0.1;
float kpRightME = 2.70;
float kiRightME = 0.180;
float kdRightME = 0.1;
int setpoint = 80;
float soffset = 8.10;

// Robot Movement
unsigned int oneGridDistance = 2900;
unsigned int turnGridDistance = 14800;  //ori=14600
unsigned int turnRightDistance = 15000; //ori=14600
unsigned int moveSpeed = 300;
unsigned int turnSpeed = 250;

// Auto-Cali Parameters
float MIN_DISTANCE_LEFT_CALIBRATE = 2.0;       // distance away from left wall to run left cali
float MIN_DISTANCE_CALIBRATE = 12;             // distance away from obstacle to trigger calibration
float ANGLE_CALIBRATION_THRESHOLD = 1.0;       // error within this value not trigger calibration
float LEFT_ANGLE_CALIBRATION_THRESHOLD = 0.10; // error within this value not trigger calibration
float FRONT_SENSORS_DISTANCE_THRESHOLD[2] = { 6.0 / 2, 7.0 / 2};  // distance away from obstacle after cali
float LEFT_SENSORS_DISTANCE_THRESHOLD[2] = {1.0, 3.0}; //{2.5/2, 3.5/2};

// Delay
int moveForwardDelay = 15; // Original = 4450
int turnDelay = 5;         // Original = 4700
int explorationDelay = 0;  // Original = 125, try 2.5
int calibrationDelay = 10; // Not used

// COUNTER
int COUNT_MOVE = 5; //counter the num of forward movement

// Sensor Range
float SRFRONT_1_RANGE[3] = {12.13, 22.10, 37.55}; // {12.10, 22.50, 24.95}
//float SRFRONT_2_RANGE[3] = {13.82, 24.51, 36.02}; //{12.80, 21.70, 24.95}
float SRFRONT_2_RANGE[3] = {12.20, 22.20, 36.02};               //{12.80, 21.70, 24.95}
float SRFRONT_3_RANGE[3] = {11.75, 23.10, 40.75};               //{12.10, 22.52, 30.72}
float SRLEFT_1_RANGE[3] = {13.00, 23.30, 34.17};                //{13.63, 20.60, 27.52}
float SRLEFT_2_RANGE[3] = {12.70, 23.20, 32.16};                //{13.63, 20.60, 26.45}
float LRRIGHT_1_RANGE[5] = {12.20, 22.70, 32.80, 44.04, 51.21}; //{13.90, 20.55, 29.54}

// Initialisation
DualVNH5019MotorShield md;

//SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 1363.2, 2.14817, -140.601); // m, b, k
//// SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 5709.05, 8.50633, -3.04403);
//SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 5709.05, 8.50633, -3.04403);
//SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5850.23, 9.47195, -11.1088);
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4237.64, -49.8888, 8.66356);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 6313.93, 16.0381, 9.38024);
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5170.73, -15.4715, 10.4833);
//SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 5285.29, 8.24699, -14.8405);
//SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 5440.65, 7.79677, -2.86093);
//SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 94637.7, 83.5766, 541.154);
SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 5285.29, -14.8405, 8.24699);
SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 5440.65, -2.86093, 7.79677);
SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 94637.7, 541.154, 83.5766);

// Parameters Declaration
// PID Calculation
unsigned long startTimeRightME = 0;
unsigned long endTimeRightME = 0;
unsigned long timeTakenRightME = 0;
unsigned long startTimeLeftME = 0;
unsigned long endTimeLeftME = 0;
unsigned long timeTakenLeftME = 0;
float RPMRight = 0;
float RPMLeft = 0;
int counterRight = 0;
int counterLeft = 0;

float k1LeftME = 0;
float k2LeftME = 0;
float k3LeftME = 0;
float k1RightME = 0;
float k2RightME = 0;
float k3RightME = 0;
float errorLeftME = 0;
float previousErrorLeftME = 0;
float previousPreviousErrorLeftME = 0;
float errorRightME = 0;
float previousErrorRightME = 0;
float previousPreviousErrorRightME = 0;
float PIDOutputLeftME = 0;
float previousPIDOutputLeftME = 0;
float PIDOutputRightME = 0;
float previousPIDOutputRightME = 0;
unsigned int distanceToMove = 0;
unsigned int totalDistance = 0;

// Sensors
String sensorOutput = "";
int counter = 0;
int sensorSampleSize = 10; //FOR TESTING IS 50
RunningMedian srSensorFront1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront3DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian lrSensorRight1DistanceRM = RunningMedian(sensorSampleSize);
float srSensorFront1Distance = 0;
float srSensorFront2Distance = 0;
float srSensorFront3Distance = 0;
float srSensorLeft1Distance = 0;
float srSensorLeft2Distance = 0;
float lrSensorRight1Distance = 0;
int srSensorFront1DistanceOutput = 0;
int srSensorFront2DistanceOutput = 0;
int srSensorFront3DistanceOutput = 0;
int srSensorLeft1DistanceOutput = 0;
int srSensorLeft2DistanceOutput = 0;
int lrSensorRight1DistanceOutput = 0;

// Calibrations
float frontSensor1ToWall = 0;
float frontSensor2ToWall = 0;
float frontSensor3ToWall = 0;
float leftSensor1ToWall = 0;
float leftSensor2ToWall = 0;
float calibrationAngleError = 0;

// Others
String sRead = "";
String algo_c = "";
String test_c = "";
boolean explorationFlag = false;
boolean fastestPathFlag = false;
// Testing Parameters (To Remove)
RunningMedian analogReadings = RunningMedian(sensorSampleSize);
RunningMedian analogReadings2 = RunningMedian(sensorSampleSize);
RunningMedian analogReadings3 = RunningMedian(sensorSampleSize);
String debugOutput = "";

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
  //  getSensorsDistanceRM(sensorSampleSize);
  //  debugSensorDistance();
  if (Serial.available() > 0)
  {
    sRead = char(Serial.read());
    if (sRead == "E")
    {
      explorationFlag = true;
      exploration();
    }
    else if (sRead == "F")
    {
      fastestPathFlag = true;
      //fastestPath();
    }
  }
}

// ==================== Modes ====================
void exploration()
{
  restartPID();
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag)
  {
    if (Serial.available() > 0)
    {
      sRead = Serial.readString();
//      sRead.trim(); // to remove \n, to integrate with algo use explorer-latest
      test_c = sRead.substring(0, 2);
      algo_c = sRead.substring(0, 1);
      sRead = sRead.substring(2);
      // Serial.println(sRead);
      // Serial.println(algo_c);
      // Testing Commands
      if (test_c == "11")
      {
        MIN_DISTANCE_CALIBRATE = sRead.toFloat();
      }
      else if (test_c == "12")
      {
        ANGLE_CALIBRATION_THRESHOLD = sRead.toFloat();
      }
      else if (test_c == "13")
      {
        LEFT_ANGLE_CALIBRATION_THRESHOLD = sRead.toFloat();
      }
      else if (test_c == "14")
      {
        kpLeftME = sRead.toFloat();
      }
      else if (test_c == "15")
      {
        kiLeftME = sRead.toFloat();
      }
      else if (test_c == "16")
      {
        kdLeftME = sRead.toFloat();
      }
      else if (test_c == "17")
      {
        kpRightME = sRead.toFloat();
      }
      else if (test_c == "18")
      {
        kiRightME = sRead.toFloat();
      }
      else if (test_c == "19")
      {
        kdRightME = sRead.toFloat();
      }
      else if (test_c == "20")
      {
        oneGridDistance = sRead.toInt();
      }
      else if (test_c == "21")
      {
        turnGridDistance = sRead.toInt();
      }
      else if (test_c == "22")
      {
        moveForwardDelay = sRead.toInt();
      }
      else if (test_c == "23")
      {
        turnDelay = sRead.toInt();
      }
      else if (test_c == "24")
      {
        explorationDelay = sRead.toInt();
      }
      else if (test_c == "25")
      {
        calibrationDelay = sRead.toInt();
      }
      else if (test_c == "26")
      {
        FRONT_SENSORS_DISTANCE_THRESHOLD[0] = sRead.toFloat();
      }
      else if (test_c == "27")
      {
        FRONT_SENSORS_DISTANCE_THRESHOLD[1] = sRead.toFloat();
      }
      else if (test_c == "28")
      {
        turnRightDistance = sRead.toInt();
      }
      //      else if(test_c == "28"){
      //        LEFT_SENSORS_DISTANCE_THRESHOLD[0] = sRead.toFloat();
      //      }else if(test_c == "29"){
      //        LEFT_SENSORS_DISTANCE_THRESHOLD[1] = sRead.toFloat();
      //      }
      else if (test_c == "30")
      {
        SRFRONT_1_RANGE[0] = sRead.toFloat();
      }
      else if (test_c == "31")
      {
        SRFRONT_1_RANGE[1] = sRead.toFloat();
      }
      else if (test_c == "32")
      {
        SRFRONT_2_RANGE[0] = sRead.toFloat();
      }
      else if (test_c == "33")
      {
        SRFRONT_2_RANGE[1] = sRead.toFloat();
      }
      else if (test_c == "34")
      {
        SRFRONT_3_RANGE[0] = sRead.toFloat();
      }
      else if (test_c == "35")
      {
        SRFRONT_3_RANGE[1] = sRead.toInt();
      }
      else if (test_c == "36")
      {
        COUNT_MOVE = sRead.toFloat();
      }
      else if (test_c == "37")
      {
        moveSpeed = sRead.toInt();
      }
      else if (test_c == "38")
      {
        turnSpeed = sRead.toInt();
      }
      else if (test_c == "39")
      {
        setpoint = sRead.toInt();
      }
      else if (test_c == "40")
      {
        //  Serial.println(sRead);
        soffset = sRead.toFloat();
      }
      else if (test_c == "DC")
      {
        delay(10);
        calibrateFrontDistance(0);
        delay(10);
      }
      else if (test_c == "AC")
      {
        delay(10);
        calibrateFrontAngle();
        delay(10);
      }
      else if (test_c == "NU")
      {
        for (int i = 0; i < COUNT_MOVE; i++)
        {
          goStraightNGrids(1);
          getSensorsDistanceRM(sensorSampleSize);
          delay(explorationDelay);
          debugSensorDistance();
          //debugPID();
          //debugDelay();
          restartPID();
        }
      }
      // Algo Commands
      else if (algo_c == "U")
      {
        goStraightNGrids(1);
        //  calibrateLeftDistance();
        delay(explorationDelay);
        //        calibrateLeftAngle();
        //        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        //restartPID();
      }
      else if (algo_c == "2")
      {
        goStraightNGrids(2);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        restartPID();
      }
      else if (algo_c == "3")
      {
        goStraightNGrids(3);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        restartPID();
      }
      else if (algo_c == "4")
      {
        goStraightNGrids(4);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        restartPID();
      }
      else if (algo_c == "5")
      {
        goStraightNGrids(5);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        restartPID();
      }
      else if (algo_c == "9")
      {
        goStraightNGrids(5);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        restartPID();
      }
      else if (algo_c == "L")
      {
        turnLeftOneGrid();
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        debugSensorDistance();
        restartPID();
      }
      else if (algo_c == "R")
      {
        turnRightOneGrid();
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        debugSensorDistance();
        restartPID();
      }
      else if (algo_c == "D")
      {
        turnRightOneGrid();
        delay(explorationDelay);
        restartPID();
        turnRightOneGrid();
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
        restartPID();
      }
      else if (algo_c == "Z")
      {
        //        getSensorsVoltageRM(sensorSampleSize);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
      }
      else if (algo_c == "#")
      {
        explorationFlag = false;
        return;
      }
      else if (algo_c == "C")
      {
        calibrate();
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (algo_c == "B")
      {
        calibrateLeftDistance();
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (algo_c == "P")
      {
        calibrateLeftAngle();
      }
    }
  }
}
// ==================== fastestPath() ====================

// ==================== PID Calculation ====================

void leftEncoderInc()
{
  counterLeft++;
  if (counterLeft == 1)
  {
    startTimeLeftME = micros();
  }
  else if (counterLeft == 26)
  {
    endTimeLeftME = micros();
    timeTakenLeftME = (endTimeLeftME - startTimeLeftME) / 25;
    RPMLeft = calculateRPM(timeTakenLeftME);
    counterLeft = 0;
  }
}

void rightEncoderInc()
{
  counterRight++;
  if (counterRight == 1)
  {
    startTimeRightME = micros();
  }
  else if (counterRight == 26)
  {
    endTimeRightME = micros();
    timeTakenRightME = (endTimeRightME - startTimeRightME) / 25;
    RPMRight = calculateRPM(timeTakenRightME);
    counterRight = 0;
  }
}

double calculateRPM(unsigned long time)
{
  if (time == 0)
  {
    return 0;
  }
  else
  {
    return 60 / (time * 562.25 / 1000000);
  }
}

void PIDCalculation(float kpLeftME, float kiLeftME, float kdLeftME, float kpRightME, float kiRightME, float kdRightME, int setpoint)
{
  // Calculate Digital PID Parameters
  k1LeftME = kpLeftME + kiLeftME + kdLeftME;
  k2LeftME = -kpLeftME - (2 * kdLeftME);
  k3LeftME = kdLeftME;
  k1RightME = kpRightME + kiRightME + kdRightME;
  k2RightME = -kpRightME - (2 * kdRightME);
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
  if (PIDOutputLeftME > 4 || PIDOutputRightME > 4)
  {
    restartPID();
  }
}

void restartPID()
{
  RPMLeft = RPMRight = 0;
  PIDOutputLeftME = PIDOutputRightME = 0;
  previousPIDOutputLeftME = previousPIDOutputRightME = 0;
  errorLeftME = errorRightME = 0;
  previousErrorLeftME = previousErrorRightME = 0;
  previousPreviousErrorLeftME = previousPreviousErrorRightME = 0;
}

// ==================== Robot Movement ====================

void goStraightNGrids(int n)
{
  //  Serial.println("goStraightNGrids");
  distanceToMove = n * oneGridDistance;
  totalDistance = 0;
  while (1)
  {
    if (totalDistance >= distanceToMove)
    {
      md.setBrakes(350, 350);
      break;
    }
    else
    {
      moveForward();
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void moveForward()
{
  //PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
  // why PIDOutputRightME is the first?
  //  md.setSpeeds(PIDOutputRightME * moveSpeed, PIDOutputLeftME * moveSpeed);
  md.setSpeeds(moveSpeed, moveSpeed + soffset);
  delay(moveForwardDelay);
}

void turnLeftOneGrid()
{
  totalDistance = 0;
  while (1)
  {
    if (totalDistance >= turnGridDistance)
    {
      md.setBrakes(400, 400);
      break;
    }
    else
    {
      md.setSpeeds(turnSpeed, -turnSpeed);
      //      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      //      md.setSpeeds(PIDOutputRightME * 150, -PIDOutputLeftME * 150);
      delay(turnDelay);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void turnRightOneGrid()
{
  totalDistance = 0;
  while (1)
  {
    if (totalDistance > turnRightDistance)
    {
      md.setBrakes(400, 400);
      break;
    }
    else
    {
      md.setSpeeds(-turnSpeed, turnSpeed);
      //      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      //      md.setSpeeds(-PIDOutputRightME * 150, PIDOutputLeftME * 150);
      delay(turnDelay);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

// ==================== Sensors ====================

// void getSensorsVoltageRM(int n)
// {
//   counter = 0;
//   analogReadings.clear();
//   analogReadings2.clear();
//   for (counter = 0; counter < n; counter++) {
// //        analogReadings.add(analogRead(srSensorFront1));
// //        analogReadings2.add(analogRead(srSensorFront2));
// //        analogReadings3.add(analogRead(srSensorFront3));
// //        analogReadings.add(analogRead(srSensorLeft1));
// //        analogReadings2.add(analogRead(srSensorLeft2));
//     //    analogReadings.add(analogRead(lrSensorRight1));
//   }
//   Serial.println("===== Median ====");
// //  Serial.println(analogReadings.getMedian());
// //  Serial.println(analogReadings2.getMedian());
// //  Serial.println(analogReadings3.getMedian());

// //  Serial.println("===== Average =====");
// //  Serial.println(analogReadings.getAverage(5));
// //  Serial.println(analogReadings2.getAverage(5));
// }

//void getSensorsVoltageRM1(int n)
//{
//  float allMediansBoth[50];
//  float allMediansS1[20];
//  float allMediansS2[20];
//  for (i = 0; i < 20; i++){
//    //pause until key pressed
//    counter = 0;
//
//    analogReadings.clear();
//    analogReadings2.clear();
//    for (counter = 0; counter < n; counter++) {
//  //    analogReadings.add(analogRead(srSensorFront1));
//  //    analogReadings2.add(analogRead(srSensorFront2));
//  //    analogReadings3.add(analogRead(srSensorFront3));
//      analogReadings.add(analogRead(srSensorLeft1));
//      analogReadings2.add(analogRead(srSensorLeft2));
//  //      analogReadings.add(analogRead(lrSensorRight1));
//    }
//    allMediansS1[i] = analogReadings.getMedian();
//    allMediansS2[i] = analogReadings2.getMedian();
//    Serial.println("===== Median ====");
//    Serial.println(analogReadings.getMedian());
//    Serial.println(analogReadings2.getMedian());
//    Serial.println("===== Average =====");
//    Serial.println(analogReadings.getAverage(5));
//    Serial.println(analogReadings2.getAverage(5));
//  }
//  Serial.println(allMediansS1);
//  Serial.println(allMediansS2);
//
//  allMediansBoth = [allMediansS1, allMediansS2];
//   Serial.println("allMediansBoth is-----------");
//   Serial.println(allMediansBoth);
//
//}

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
  for (counter = 0; counter < n; counter++)
  {
    srSensorFront1DistanceRM.add(SRSensorFront1.distance());
    srSensorFront2DistanceRM.add(SRSensorFront2.distance());
    srSensorFront3DistanceRM.add(SRSensorFront3.distance());
    srSensorLeft1DistanceRM.add(SRSensorLeft1.distance());
    srSensorLeft2DistanceRM.add(SRSensorLeft2.distance());
    lrSensorRight1DistanceRM.add(LRSensorRight1.distance());
  }
  // Short Range Sensor Front 1
  srSensorFront1Distance = srSensorFront1DistanceRM.getAverage(5);
  if (srSensorFront1Distance <= SRFRONT_1_RANGE[0])
  {
    srSensorFront1DistanceOutput = -1;
  }
  else if (srSensorFront1Distance <= SRFRONT_1_RANGE[1])
  {
    srSensorFront1DistanceOutput = 1;
  }
  //  else if (srSensorFront1Distance <= SRFRONT_1_RANGE[2]) {
  //    srSensorFront1DistanceOutput = 2;
  //  }
  else
  {
    srSensorFront1DistanceOutput = 0;
  }
  // Short Range Sensor Front 2
  srSensorFront2Distance = srSensorFront2DistanceRM.getAverage(5);
  if (srSensorFront2Distance <= SRFRONT_2_RANGE[0])
  {
    srSensorFront2DistanceOutput = -1;
  }
  else if (srSensorFront2Distance <= SRFRONT_2_RANGE[1])
  {
    srSensorFront2DistanceOutput = 1;
  }
  //  else if (srSensorFront2Distance <= SRFRONT_2_RANGE[2]) {
  //    srSensorFront2DistanceOutput = 2;
  //  }
  else
  {
    srSensorFront2DistanceOutput = 0;
  }
  // Short Range Sensor Front 3
  srSensorFront3Distance = srSensorFront3DistanceRM.getAverage(5);
  if (srSensorFront3Distance <= SRFRONT_3_RANGE[0])
  {
    srSensorFront3DistanceOutput = -1;
  }
  else if (srSensorFront3Distance <= SRFRONT_3_RANGE[1])
  {
    srSensorFront3DistanceOutput = 1;
  }
  //  else if (srSensorFront3Distance <= SRFRONT_3_RANGE[2]) {
  //    srSensorFront3DistanceOutput = 2;
  //  }
  else
  {
    srSensorFront3DistanceOutput = 0;
  }
  // Short Range Sensor Left 1
  srSensorLeft1Distance = srSensorLeft1DistanceRM.getAverage(5);
  if (srSensorLeft1Distance <= SRLEFT_1_RANGE[0])
  {
    srSensorLeft1DistanceOutput = -1;
  }
  else if (srSensorLeft1Distance <= SRLEFT_1_RANGE[1])
  {
    srSensorLeft1DistanceOutput = 1;
  }
  //  else if (srSensorLeft1Distance <= SRLEFT_1_RANGE[2]) {
  //    srSensorLeft1DistanceOutput = 2;
  //  }
  else
  {
    srSensorLeft1DistanceOutput = 0;
  }
  // Short Range Sensor Left 2
  srSensorLeft2Distance = srSensorLeft2DistanceRM.getAverage(5);
  if (srSensorLeft2Distance <= SRLEFT_2_RANGE[0])
  {
    srSensorLeft2DistanceOutput = -1;
  }
  else if (srSensorLeft2Distance <= SRLEFT_2_RANGE[1])
  {
    srSensorLeft2DistanceOutput = 1;
  }
  //  else if (srSensorLeft2Distance <= SRLEFT_2_RANGE[2]) {
  //    srSensorLeft2DistanceOutput = 2;
  //  }
  else
  {
    srSensorLeft2DistanceOutput = 0;
  }
  // Long Range Sensor Right 1
  lrSensorRight1Distance = lrSensorRight1DistanceRM.getAverage(5);
  if (lrSensorRight1Distance <= LRRIGHT_1_RANGE[0])
  {
    lrSensorRight1DistanceOutput = -1;
  }
  else if (lrSensorRight1Distance <= LRRIGHT_1_RANGE[1])
  {
    lrSensorRight1DistanceOutput = 1;
  }
  else if (lrSensorRight1Distance <= LRRIGHT_1_RANGE[2])
  {
    lrSensorRight1DistanceOutput = 2;
  }
  else
  {
    lrSensorRight1DistanceOutput = 0;
  }
  sensorOutput = sensorOutput + srSensorFront1DistanceOutput + "|" + srSensorFront2DistanceOutput + "|" + srSensorFront3DistanceOutput + "|" + srSensorLeft1DistanceOutput + "|" + srSensorLeft2DistanceOutput + "|" + lrSensorRight1DistanceOutput;
  Serial.println(sensorOutput);
}

// ==================== Calibration ====================

void calibrate()
{
  float frontSensor2ToWall = SRSensorFront2.distance(); //getDistance(2);
  // frontSensor2ToWall should be slightly smaller than the SRFRONT_2_RANGE[1]
  if (frontSensor2ToWall > SRFRONT_2_RANGE[1] - 1)
    return;
  delay(calibrationDelay);
  calibrateFrontAngle();
  delay(calibrationDelay);
  calibrateFrontDistance(0);
  delay(calibrationDelay);
  calibrateFrontAngle();
}

void calibrateFrontDistance(float offset)
{
  frontSensor2ToWall = 0;
  float error;
  while (1)
  {
    //Original
    frontSensor2ToWall = SRSensorFront2.distance();
    error = 3.5 - frontSensor2ToWall;



    //abort if within margin of error
    if (frontSensor2ToWall >= FRONT_SENSORS_DISTANCE_THRESHOLD[0] && frontSensor2ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[1])
      break;

    if (frontSensor2ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[0])
    {
      //reverse normally
      //      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 30);
      //      md.setSpeeds(PIDOutputRightME * -100, PIDOutputLeftME * -100);
      md.setSpeeds(-120, -120);
      delay(abs(error) * 50); //100
      md.setBrakes(400, 400);
    }

    else if (frontSensor2ToWall > FRONT_SENSORS_DISTANCE_THRESHOLD[1])
    {
      //move forward
      //moveForward(100,0.9);
      // Serial.println("Moving forwards");
      //      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 30);
      //      md.setSpeeds(PIDOutputRightME * 100, PIDOutputLeftME * 100);
      md.setSpeeds(120, 120);
      delay(abs(error) * 50); //100
      md.setBrakes(400, 400);
    }
  }
}

void calibrateFrontSensorsAngle(float error)
{
  if (error > ANGLE_CALIBRATION_THRESHOLD)
  {
    //    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    //    md.setSpeeds(PIDOutputRightME * 50, -PIDOutputLeftME * 50);
    md.setSpeeds(100, -100);
    delay(abs(error * 25));
    md.setBrakes(400, 400);
  }
  else if (error < -ANGLE_CALIBRATION_THRESHOLD)
  {
    //    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    //    md.setSpeeds(-PIDOutputRightME * 50, PIDOutputLeftME * 50);
    md.setSpeeds(-100, 100);
    delay(abs(error * 25));
    md.setBrakes(400, 400);
  }
}

void calibrateLeftSensorsAngle(float error)
{
  if (error > LEFT_ANGLE_CALIBRATION_THRESHOLD)
  {
    //    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    //    md.setSpeeds(PIDOutputRightME * 50, -PIDOutputLeftME * 50);
    md.setSpeeds(100, -100);
    delay(abs(error * 25));
    md.setBrakes(400, 400);
  }
  else if (error < -LEFT_ANGLE_CALIBRATION_THRESHOLD)
  {
    //    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    //    md.setSpeeds(-PIDOutputRightME * 50, PIDOutputLeftME * 50);
    md.setSpeeds(-100, 100);
    delay(abs(error * 25));
    md.setBrakes(400, 400);
  }
}
void calibrateLeftAngle()
{
  //  Serial.println("calibrateLeftAngle");
  leftSensor1ToWall = SRSensorLeft1.distance();
  leftSensor2ToWall = SRSensorLeft2.distance();
  if (leftSensor1ToWall > MIN_DISTANCE_LEFT_CALIBRATE + 5 || leftSensor2ToWall > MIN_DISTANCE_LEFT_CALIBRATE + 5)
  {
    return;
  }
  int counter = 0;
  float calibrationAngleError = 0;
  double angleDist = 0;
  while (counter < 20)
  {
    leftSensor1ToWall = SRSensorLeft1.distance();
    leftSensor2ToWall = SRSensorLeft2.distance();
    calibrationAngleError = leftSensor1ToWall - leftSensor2ToWall;
    counter++;
    if (abs(calibrationAngleError) <= LEFT_ANGLE_CALIBRATION_THRESHOLD)
    {
      break;
    }
    calibrateLeftSensorsAngle(calibrationAngleError);
  }
}
void calibrateFrontAngle()
{
  int counter = 0;
  frontSensor1ToWall = 0;
  frontSensor3ToWall = 0;
  float calibrationAngleError = 0;
  double angleDist = 0;
  while (counter < 15)
  {
    frontSensor1ToWall = SRSensorFront1.distance();
    frontSensor3ToWall = SRSensorFront3.distance();
    calibrationAngleError = frontSensor1ToWall - frontSensor3ToWall;
    counter++;
    if (abs(calibrationAngleError) <= ANGLE_CALIBRATION_THRESHOLD)
    {
      break;
    }
    calibrateFrontSensorsAngle(calibrationAngleError);
    md.setBrakes(400, 400);
  }
}

void calibrateLeftDistance()
{
  // Serial.println("=== calibrateLeftDistance ===");
  // Function that implement the logic when to turn left 90degree, and then call calibrateFrontDistance.
  leftSensor1ToWall = 0;
  leftSensor2ToWall = 0;
  leftSensor1ToWall = SRSensorLeft1.distance();
  leftSensor2ToWall = SRSensorLeft2.distance();
  // Serial.println(leftSensor1ToWall);
  // Serial.println(leftSensor2ToWall);
  float diff = abs(leftSensor1ToWall - leftSensor2ToWall);
  if (leftSensor1ToWall > MIN_DISTANCE_LEFT_CALIBRATE && leftSensor2ToWall > MIN_DISTANCE_LEFT_CALIBRATE)
  {
    return;
  }
    Serial.println("Running before calibrateLeftAngle");
    calibrateLeftAngle();
    Serial.println("Running after calibrateLeftAngle");
    //abort if within margin of error
    if ((leftSensor1ToWall >= LEFT_SENSORS_DISTANCE_THRESHOLD[0] && leftSensor1ToWall < LEFT_SENSORS_DISTANCE_THRESHOLD[1]) || (leftSensor2ToWall >= LEFT_SENSORS_DISTANCE_THRESHOLD[0] && leftSensor2ToWall < LEFT_SENSORS_DISTANCE_THRESHOLD[1]))
    {
      return;
    }
      delay(calibrationDelay);
      turnLeftOneGrid();
      delay(calibrationDelay);
      calibrate();
      delay(calibrationDelay);
      turnRightOneGrid();
      delay(calibrationDelay);
      calibrateLeftAngle();
}

// ==================== Debug (To Remove) ====================

void debugSensorDistance()
{
  debugOutput = "d";
  debugOutput = debugOutput + srSensorFront1Distance + " | " + srSensorFront2Distance + " | " + srSensorFront3Distance + " | " + srSensorLeft1Distance + " | " + srSensorLeft2Distance + " | " + lrSensorRight1Distance;
  Serial.println(debugOutput);
}

void debugPID()
{
  debugOutput = "d";
  debugOutput = debugOutput + "Left PID = " + PIDOutputLeftME + " | Right PID = " + PIDOutputRightME;
  //  Serial.println(debugOutput);
}

void debugDelay()
{
  debugOutput = "d";
  debugOutput = debugOutput + "moveForwardDelay = " + moveForwardDelay + " | turnDelay = " + turnDelay + " | explorationDelay = " + explorationDelay + " | calibrationDelay = " + calibrationDelay;
  //  Serial.println(debugOutput);
}
