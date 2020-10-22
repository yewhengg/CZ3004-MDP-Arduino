// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"
// M1 = Right Motor, M2 = Left Motor

// Sensor Library
// https://github.com/guillaume-rico/SharpIR
#include "SharpIR.h"

// Enable Interrupt Library
// https://github.com/GreyGnome/EnableInterrupt
#include "EnableInterrupt.h"

// Running Median Library
// https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian
#include <RunningMedian.h>

// Parameters Definition
#define motorEncoderRight1 3
#define motorEncoderLeft1 11
#define srSensorFront1 A0     // PS1
#define srSensorFront2 A1     // PS2
#define srSensorFront3 A2     // PS3
#define srSensorLeft1 A3      // PS4
#define srSensorLeft2 A4      // PS5
#define lrSensorRight1 A5     // PS6
#define SRSensor_Model 1080
#define LRSensor_Model 20150

// Motor Initialisation
DualVNH5019MotorShield md;
// Sensors Initialisation
//SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 3509.11, 10.1626, -83.674);
//SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 5305.81, 11.1486, -34.9659);
//SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5122.21, 12.9189, -31.1762);
//SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 5177.84, 11.0427, -21.9181);
//SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 7840.02, 14.9919, 43.0052);
//SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 302904, 169.317, 1249.89);
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4237.64, -49.8888, 8.66356);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 6313.93, 16.0381, 9.38024);
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5170.73, -15.4715, 10.4833);
SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 5285.29, -14.8405,8.24699 );
SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 5440.65, -2.86093, 7.79677 );
SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 94637.7, 541.154, 83.5766); 

// Auto-Cali Parameters
float MIN_DISTANCE_LEFT_CALIBRATE = 3.3;       // distance away from left wall to run left cali
float MIN_DISTANCE_CALIBRATE = 12;             // distance away from obstacle to trigger calibration
float ANGLE_CALIBRATION_THRESHOLD = 0.25; //1.0;       // error within this value not trigger calibration
float LEFT_ANGLE_CALIBRATION_THRESHOLD = 0.05; // error within this value not trigger calibration
float FRONT_SENSORS_DISTANCE_THRESHOLD[2] = { 4.0 , 4.5};  // distance away from obstacle after cali
float LEFT_SENSORS_DISTANCE_THRESHOLD[2] = {2.8, 4.5}; //{2.5/2, 3.5/2};

//Robot Movement
unsigned int forwardTicks = 272; //273; //285;
unsigned int leftTurnTicks = 383;// 387 - 6.3V 383 - 6.4V
unsigned int rightTurnTicks = 385; //385;//387 - 6.3V 385 - 6.4V

//pid
double kpL = 24.7; //20;
double kiL = 0.48; //0.065;
double kdL = 3; //4;
double kpR = 24.7 ;//V=6.4V : 24.7;
double kiR = 0.2;
double kdR = 3;
double kpDiff = 5.0; //V=6.4V : 5.0
double kiDiff = 0.044; //0.045
double kdDiff = 10;

// Parameters Declaration
// Interrupt + PID
volatile long leftTicks, rightTicks;
volatile long rightCounterReduce = 0;
double previousLeftTicks = 0;
boolean moveForwardFlag = false;
double RPMLeft = 0;
double RPMRight = 0;
double previousRPMLeft = 0;
double previousRPMRight = 0;
double integralLeft = 0;
double integralRight = 0;
double integralDiff = 0;
double setpoint = 120;
// Motor
int movingSetpoint = 75;
float soffset = 8.10;

// Sensor
int counter = 0;
String sensorOutput = "";
int sensorSampleSize = 10;
RunningMedian srSensorFront1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorFront3DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft1DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian srSensorLeft2DistanceRM = RunningMedian(sensorSampleSize);
RunningMedian lrSensorRight1DistanceRM = RunningMedian(sensorSampleSize);
// Sensor Range
float SRFRONT_1_RANGE[3] = {11.10, 19.82, 37.55}; // {11.10, 22.10, 37.55}; 
float SRFRONT_2_RANGE[3] = {11.50, 20.90, 36.02};               //{11.10, 22.20, 36.02}; 
float SRFRONT_3_RANGE[3] = {10.60, 20.20, 40.75};               //{11.10, 23.10, 40.75};
float SRLEFT_1_RANGE[3] = {11.52, 22.30, 34.17};                //{13.63, 20.60, 27.52}
float SRLEFT_2_RANGE[3] = {11.70, 23.20, 32.16};                //{13.63, 20.60, 26.45}
float LRRIGHT_1_RANGE[5] = {12.52, 22.70, 31.00, 44.04, 51.21}; //{13.90, 20.55, 29.54}
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
double newFrontAngleThreshold[2] = {0.55, 0.80};
double newFrontSensor1AngleThreshold[2] = {4.22, 4.32};
double newFrontSensor3AngleThreshold[2] = {3.13, 3.23};

// Others
String cc = "";
String sRead = "";
String algo_c = "";
String test_c = "";
boolean explorationFlag = false;
boolean fastestPathFlag = false;

// Delay
int moveForwardDelay = 15; // Original = 4450
int turnDelay = 5;         // Original = 4700
int explorationDelay = 100;  // Original = 125, try 2.5
int calibrationDelay = 10; // Not used
// Testing Parameters (To Remove)
RunningMedian analogReadings = RunningMedian(sensorSampleSize);
RunningMedian analogReadings2 = RunningMedian(sensorSampleSize);
RunningMedian analogReadings3 = RunningMedian(sensorSampleSize);
String debugOutput = "";

// ==================== Setup ====================

void setup()
{
  Serial.begin(115200);
  md.init();
  pinMode(motorEncoderLeft1, INPUT);
  pinMode(motorEncoderRight1, INPUT);
  enableInterrupt(motorEncoderLeft1, leftEncoderInc, RISING);
  enableInterrupt(motorEncoderRight1, rightEncoderInc, RISING);
  delay(100);
}


// ==================== Loop ====================

void loop()
{
  //goStraightNGrids(1);
  //delay(100);
  if (Serial.available() > 0) {
    cc = char(Serial.read());
    if (cc == "E") {
      explorationFlag = true;
      exploration();
    } else if (cc == "F") {
      fastestPathFlag = true;
      fastestPath();
    }
  }
}

// ==================== Modes ====================

void exploration()
{
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag) {
    if (Serial.available() > 0) {
      sRead = Serial.readString();
//      sRead.trim(); // to remove \n, 
// To integrate with algo use explorer-latest-ticks one
      test_c = sRead.substring(0, 2);
      algo_c = sRead.substring(0, 1);
      sRead = sRead.substring(2);
      Serial.println(test_c);
      Serial.println("|");
      Serial.println(algo_c);
      Serial.println("|");
      Serial.println(sRead);
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
      /*
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
      */
      else if (test_c == "20")
      {
        forwardTicks = sRead.toInt();
      }
      else if (test_c == "21")
      {
        leftTurnTicks = sRead.toInt();
      }
      else if (test_c == "22")
      {
        rightTurnTicks = sRead.toInt();
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
      else if(test_c == "28"){
        LEFT_SENSORS_DISTANCE_THRESHOLD[0] = sRead.toFloat();
      }else if(test_c == "29"){
        LEFT_SENSORS_DISTANCE_THRESHOLD[1] = sRead.toFloat();
      }
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
        SRFRONT_3_RANGE[1] = sRead.toFloat();
      }
//      else if (test_c == "35")
//      {
//       SRFRONT_3_RANGE[1] = sRead.toFloat();
//      }
//            else if (test_c == "35")
//      {
//        SRFRONT_3_RANGE[1] = sRead.toFloat();
//      }
      if (algo_c == "U") {
        goStraightNGrids(1);
        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
      } else if (algo_c == "L") {
        turnLeftOneGrid();
        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (algo_c == "R") {
        turnRightOneGrid();
        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (algo_c == "Z") {
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
      } else if (algo_c == "C"){
        // runned when all frontSensors gives -1.
        calibrate();
        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      } 
//      else if (cc == "D") {
//        // runned when only the middle frontSensor gives -1.
//        calibrateFrontDistance(0);
//        delay(explorationDelay);
//        getSensorsDistanceRM(sensorSampleSize);
//      } 
      else if (algo_c == "B"){
        calibrateLeftDistance();
        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (algo_c == "P"){
        calibrateLeftAngle();
        delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      } else if (algo_c == "#") {
        explorationFlag = false;
      }  
    }
  }
}



void fastestPath(){
  while(fastestPathFlag){
    if(Serial.available() > 0){
      cc = char(Serial.read());
      if(cc == "1"){
        goStraightNGrids(1);
      } else if(cc == "2"){
        goStraightNGrids(2);
      } else if(cc == "3"){
        goStraightNGrids(3);
      } else if(cc == "4"){
        goStraightNGrids(4);
      } else if(cc == "5"){
        goStraightNGrids(5);
      } else if(cc == "6"){
        goStraightNGrids(6);
      } else if(cc == "7"){
        goStraightNGrids(7);
      } else if(cc == "8"){
        goStraightNGrids(8);
      } else if(cc == "9"){
        goStraightNGrids(9);
      } else if(cc == "A"){
        goStraightNGrids(10);
      } else if(cc == "B"){
        goStraightNGrids(11);
      } else if(cc == "C"){
        goStraightNGrids(12);
      } else if(cc == "D"){
        goStraightNGrids(13);
      } else if(cc == "E"){
        goStraightNGrids(14);
      } else if(cc == "F"){
        goStraightNGrids(15);
      } else if (cc == "R"){
        turnRightOneGrid();
      } else if (cc == "L"){
        turnLeftOneGrid();
      } else if (cc == "#") {
        fastestPathFlag = false;
      }
    }
  }
}
// ==================== Interrupts ====================

void leftEncoderInc()
{
  leftTicks++;
}

void rightEncoderInc()
{
  rightTicks++;
  if (moveForwardFlag) {
    if (rightCounterReduce++ == 95) {
      rightCounterReduce = 0;
      rightTicks--;
    }
  }
}

// ==================== PID ====================

void restartPID()
{
  integralLeft = 0;
  integralRight = 0;
  previousRPMLeft = 0;
  previousRPMRight = 0;
  previousLeftTicks = 0;
  moveForwardFlag = false;
  rightCounterReduce = 0;
}

double computePIDLeft(double RPM, double setpoint)
{
//  double kp;
//  double ki;
//  double kd;
  double error;
  double p;
  double i;
  double d;
  double pid;
//  kp = 20;
//  ki = 0.065;
//  kd = 4;
  error = setpoint - RPM;
  integralLeft = integralLeft + error;
  p = kpL * error;
  i = kiL * integralLeft;
  d = kdL * (previousRPMLeft - RPM);
  previousRPMLeft = RPM;
  pid = p + i + d;
  return pid;
}

double computePIDRight(double RPM, double setpoint)
{
//  double kp;
//  double ki;
//  double kd;
  double error;
  double p;
  double i;
  double d;
  double pid;
//  kp = 24.7; // 25
//  ki = 0.2;
//  kd = 3;
  error = setpoint - RPM;
  integralRight = integralRight + error;
  p = kpR * error;
  i = kiR * integralRight;
  d = kdR * (previousRPMRight - RPM);
  previousRPMRight = RPM;
  pid = p + i + d;
  return pid;
}

double computePIDDiff()
{
//  double kp;
//  double ki;
//  double kd;
  double error;
  double p;
  double i;
  double d;
  double pid;
  double timeChange;
//  kp = 9.8; // 10
//  ki = 0.1;
//  kd = 10;
  timeChange = (leftTicks - previousLeftTicks) * 1.5;
  error = leftTicks - rightTicks;
  error = error + (error * timeChange);
  integralDiff += error;
  p = kpDiff * error;
  i = kiDiff * integralDiff;
  if (timeChange == 0) {
    d = 0;
  } else {
    d = (kdDiff * (previousLeftTicks - leftTicks)) / timeChange;
  }
  previousLeftTicks = leftTicks;
  pid = p + i + d;
  return pid;
}

// ==================== Motor Movement ====================

void goStraightNGrids(int numGrids)
{
  unsigned long durationLeft;
  unsigned long durationRight;
  double totalRPMLeft;
  double totalRPMRight;
  double PIDOutputLeft;
  double PIDOutputRight;
  double PIDOutputDiff;
  double ticksToMove;
  RPMLeft = 0;
  RPMRight = 0;
  totalRPMLeft = 0;
  totalRPMRight = 0;
  PIDOutputLeft = 0;
  PIDOutputRight = 0;
  PIDOutputDiff = 0;
  leftTicks = 0;
  rightTicks = 0;
  ticksToMove = forwardTicks * numGrids;
//  if(numGrids > 1){
//    ticksToMove = 285 * pow(numGrids, 1.5); // 285
//  }
  restartPID();
  moveForwardFlag = true;
  while (leftTicks < ticksToMove && rightTicks < ticksToMove) {
    movingSetpoint = ticksToMove - leftTicks;
    if (movingSetpoint < 25) {
      PIDOutputLeft = computePIDLeft(RPMLeft, 90);
      PIDOutputRight = computePIDRight(RPMRight, 90);
    } else {
      PIDOutputLeft = computePIDLeft(RPMLeft, setpoint);
      PIDOutputRight = computePIDRight(RPMRight, setpoint);
    }
    PIDOutputDiff = computePIDDiff();
    md.setSpeeds((PIDOutputRight + PIDOutputDiff), (PIDOutputLeft - PIDOutputDiff));
//    md.setSpeeds(320,320 + soffset);
    durationLeft = 2 * pulseIn(motorEncoderLeft1, HIGH);
    RPMLeft = 60 / (562.25 * durationLeft * 0.000001);
    durationRight = 2 * pulseIn(motorEncoderRight1, HIGH);
    RPMRight = 60 / (562.25 * durationRight * 0.000001);
  }
  md.setBrakes(400, 390);
}

void turnLeftOneGrid()
{
  unsigned long durationLeft;
  unsigned long durationRight;
  double PIDOutputLeft;
  double PIDOutputRight;
  double PIDOutputDiff;
  double ticksToMove;
  RPMLeft = 0;
  RPMRight = 0;
  PIDOutputLeft = 0;
  PIDOutputRight = 0;
  PIDOutputDiff = 0;
  ticksToMove = leftTurnTicks; // initially 400
  leftTicks = 0;
  rightTicks = 0;
  restartPID();
  while (leftTicks < ticksToMove && rightTicks < ticksToMove) {
    PIDOutputLeft = computePIDLeft(RPMLeft, setpoint);
    PIDOutputRight = computePIDRight(RPMRight, setpoint);
    PIDOutputDiff = computePIDDiff();
    md.setSpeeds((PIDOutputRight + PIDOutputDiff), -1 * (PIDOutputLeft - PIDOutputDiff));
//    md.setSpeeds(320 , -320);
    durationLeft = 2 * pulseIn(motorEncoderLeft1, HIGH);
    RPMLeft = 60 / (562.25 * durationLeft * 0.000001);
    durationRight = 2 * pulseIn(motorEncoderRight1, HIGH);
    RPMRight = 60 / (562.25 * durationRight * 0.000001);
  }
  md.setBrakes(400, 360);
}

void turnRightOneGrid()
{
  unsigned long durationLeft;
  unsigned long durationRight;
  double PIDOutputLeft;
  double PIDOutputRight;
  double PIDOutputDiff;
  double ticksToMove;
  RPMLeft = 0;
  RPMRight = 0;
  PIDOutputLeft = 0;
  PIDOutputRight = 0;
  PIDOutputDiff = 0;
  ticksToMove = rightTurnTicks;
  leftTicks = 0;
  rightTicks = 0;
  restartPID();
  while (leftTicks < ticksToMove && rightTicks < ticksToMove) {
    PIDOutputLeft = computePIDLeft(RPMLeft, setpoint);
    PIDOutputRight = computePIDRight(RPMRight, setpoint);
    PIDOutputDiff = computePIDDiff();
    md.setSpeeds(-1 * (PIDOutputRight + PIDOutputDiff), (PIDOutputLeft - PIDOutputDiff));
//    md.setSpeeds(-320, 320);
    durationLeft = 2 * pulseIn(motorEncoderLeft1, HIGH);
    RPMLeft = 60 / (562.25 * durationLeft * 0.000001);
    durationRight = 2 * pulseIn(motorEncoderRight1, HIGH);
    RPMRight = 60 / (562.25 * durationRight * 0.000001);
  }
  md.setBrakes(400, 360); //(400,400);
}

// ==================== Sensors ====================

void getSensorsVoltageRM(int n)
{
  counter = 0;
  analogReadings.clear();
  analogReadings2.clear();
  analogReadings3.clear();
  for (counter = 0; counter < n; counter++) {
//    analogReadings.add(analogRead(srSensorFront1));
//    analogReadings2.add(analogRead(srSensorFront2));
//    analogReadings3.add(analogRead(srSensorFront3));
//    analogReadings.add(analogRead(srSensorLeft1));
//    analogReadings2.add(analogRead(srSensorLeft2));
//      analogReadings.add(analogRead(lrSensorRight1));
  }
//  Serial.println("===== Median ====");
//  Serial.println(analogReadings.getMedian());
//  Serial.println(analogReadings2.getMedian());
//  Serial.println(analogReadings3.getMedian());
//  Serial.println("===== Average =====");
//  Serial.println(analogReadings.getAverage(5));
//  Serial.println(analogReadings2.getAverage(5));
//  Serial.println(analogReadings3.getAverage(5));
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

void checkCaliFrontDistance(){
  frontSensor2ToWall = SRSensorFront2.distance(); //getDistance(2);
  frontSensor1ToWall = SRSensorFront1.distance();
  frontSensor3ToWall = SRSensorFront3.distance();
  
  if (frontSensor2ToWall <= SRFRONT_2_RANGE[0] - 1.0){
    calibrateFrontDistance(2);
    delay(calibrationDelay);
  } else if(frontSensor1ToWall <= SRFRONT_1_RANGE[0] - 1.0) {
    calibrateFrontDistance(1);
    delay(calibrationDelay);
  } else if(frontSensor3ToWall <= SRFRONT_3_RANGE[0] - 1.0) {
    calibrateFrontDistance(3);
    delay(calibrationDelay);
  }
  return;
}

void calibrateFrontDistance(int sensor)
{
  frontSensor2ToWall = 0;
  float dist;
  float range[2];
  float error;
  while (1)
  {
    if(sensor == 2){
      dist = SRSensorFront2.distance();
      
    } else if(sensor == 1){
      dist = SRSensorFront1.distance();
      
    } else {
      dist = SRSensorFront3.distance();
      
    }
    //Original
//    frontSensor1ToWall = SRSensorFront1.distance();
//    frontSensor2ToWall = SRSensorFront2.distance();
//    frontSensor3ToWall = SRSensorFront3.distance();
    error = FRONT_SENSORS_DISTANCE_THRESHOLD[0] - dist;
    
    //abort if within margin of error
    if (dist >= FRONT_SENSORS_DISTANCE_THRESHOLD[0] && dist < FRONT_SENSORS_DISTANCE_THRESHOLD[1]){
      
      break;
    }

    if (dist < FRONT_SENSORS_DISTANCE_THRESHOLD[0])
    {
      //move back
      md.setSpeeds(-100, -100);
      delay(abs(error) * 50); //100
      md.setBrakes(400, 400);
    }

    else if (dist > FRONT_SENSORS_DISTANCE_THRESHOLD[1])
    {
      //move forward
      md.setSpeeds(100, 100);
      delay(abs(error) * 50); //100
      md.setBrakes(400, 400);
    }
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
    md.setBrakes(400, 360);
  }
  else if (error < -LEFT_ANGLE_CALIBRATION_THRESHOLD)
  {
    //    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    //    md.setSpeeds(-PIDOutputRightME * 50, PIDOutputLeftME * 50);
    md.setSpeeds(-100, 100);
    delay(abs(error * 25));
    md.setBrakes(400, 360);
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
  while (counter < 15)
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
//void calibrateLeftAngle()
//{
//  double leftSensor1ToWall = SRSensorLeft1.distance();
//  double leftSensor2ToWall = SRSensorLeft2.distance();
//  double calibrationAngleError = leftSensor1ToWall - leftSensor2ToWall;
//  if (calibrationAngleError > LEFT_ANGLE_CALIBRATION_THRESHOLD) {
//    md.setSpeeds(50, -50);
//    while (calibrationAngleError > LEFT_ANGLE_CALIBRATION_THRESHOLD) {
//      leftSensor1ToWall = SRSensorLeft1.distance();
//      leftSensor2ToWall = SRSensorLeft2.distance();
//      Serial.println(leftSensor1ToWall);
//      Serial.println(leftSensor2ToWall);
//      calibrationAngleError = leftSensor1ToWall - leftSensor2ToWall;
//      if (abs(calibrationAngleError) <= LEFT_ANGLE_CALIBRATION_THRESHOLD) {
//        md.setBrakes(400, 400);
//      }
//    }
//  } else if (calibrationAngleError < -LEFT_ANGLE_CALIBRATION_THRESHOLD) {
//    md.setSpeeds(-50, 50);
//    while (calibrationAngleError < -LEFT_ANGLE_CALIBRATION_THRESHOLD) {
//      leftSensor1ToWall = SRSensorLeft1.distance();
//      leftSensor2ToWall = SRSensorLeft2.distance();
//      Serial.println(leftSensor1ToWall);
//      Serial.println(leftSensor2ToWall);
//      calibrationAngleError = leftSensor1ToWall - leftSensor2ToWall;
//      if (abs(calibrationAngleError) <= LEFT_ANGLE_CALIBRATION_THRESHOLD) {
//        md.setBrakes(400, 400);
//      }
//    }
//  }
//}
void calibrateFrontAngle()
{
  double frontSensor1ToWall = SRSensorFront1.distance();
  double frontSensor2ToWall = SRSensorFront2.distance();
  double calibrationAngleError = frontSensor1ToWall - frontSensor2ToWall;
  if (calibrationAngleError > ANGLE_CALIBRATION_THRESHOLD) {
    md.setSpeeds(80, -80);
    while (calibrationAngleError > ANGLE_CALIBRATION_THRESHOLD) {
      frontSensor1ToWall = SRSensorFront1.distance();
      frontSensor2ToWall = SRSensorFront2.distance();
      calibrationAngleError = frontSensor1ToWall - frontSensor2ToWall;
      if (abs(calibrationAngleError) <= ANGLE_CALIBRATION_THRESHOLD) {
        md.setBrakes(400, 400);
      }
    }
  } else if (calibrationAngleError < -ANGLE_CALIBRATION_THRESHOLD) {
    md.setSpeeds(-80, 80);
    while (calibrationAngleError < -ANGLE_CALIBRATION_THRESHOLD) {
      frontSensor1ToWall = SRSensorFront1.distance();
      frontSensor2ToWall = SRSensorFront2.distance();
      calibrationAngleError = frontSensor1ToWall - frontSensor2ToWall;
      if (abs(calibrationAngleError) <= ANGLE_CALIBRATION_THRESHOLD) {
        md.setBrakes(400, 400);
      }
    }
  }
}
//void calibrateFrontAngle()
//{
//  int counter = 0;
//  frontSensor1ToWall = 0;
//  frontSensor3ToWall = 0;
//  float calibrationAngleError = 0;
//  double angleDist = 0;
//  while (counter < 50)
//  {
//    frontSensor1ToWall = SRSensorFront1.distance();
//    frontSensor3ToWall = SRSensorFront3.distance();
//    calibrationAngleError = frontSensor1ToWall - frontSensor3ToWall;
//    
//    counter++;
////    if (frontSensor1ToWall >= newFrontSensor1AngleThreshold[0] && frontSensor1ToWall <= newFrontSensor1AngleThreshold[1] && frontSensor3ToWall >= newFrontSensor3AngleThreshold[0] && frontSensor3ToWall <= newFrontSensor3AngleThreshold[1])
////    if (abs(calibrationAngleError) >= newFrontAngleThreshold[0] && abs(calibrationAngleError) <= newFrontAngleThreshold[1])
//    if (abs(calibrationAngleError) < ANGLE_CALIBRATION_THRESHOLD)
//    {
//      break;
//    }
//    calibrateFrontSensorsAngle(calibrationAngleError);
//    md.setBrakes(400, 400);
//  }
//}

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
//  if (leftSensor1ToWall > SRLEFT_1_RANGE[0] || leftSensor2ToWall > SRLEFT_2_RANGE[0]) {
//    return;
//  }
//  calibrateLeftAngle();
  if (leftSensor1ToWall > MIN_DISTANCE_LEFT_CALIBRATE && leftSensor2ToWall > MIN_DISTANCE_LEFT_CALIBRATE )
  {
    return;
  }
//    Serial.println("Running before calibrateLeftAngle");
//    calibrateLeftAngle();
    
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
      
}

// ==================== Debug ====================

void debugSensorDistance()
{
  debugOutput = "d";
  debugOutput = debugOutput + srSensorFront1Distance + " | " + srSensorFront2Distance + " | " + srSensorFront3Distance + " | " + srSensorLeft1Distance + " | " + srSensorLeft2Distance + " | " + lrSensorRight1Distance;
  Serial.println(debugOutput);
}
