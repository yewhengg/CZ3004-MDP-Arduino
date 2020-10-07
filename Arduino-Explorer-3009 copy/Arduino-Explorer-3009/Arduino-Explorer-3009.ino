// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"
// M1 = Right Motor, M2 = Left Motor

// Sensor Library
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
#define srSensorFront1 A0     //PS1
#define srSensorFront2 A1     //PS2
#define srSensorFront3 A2     //PS3
#define srSensorLeft1 A3      //PS4
#define srSensorLeft2 A4      //PS5
#define lrSensorRight1 A5     //PS6
#define SRSensor_Model 1080
#define LRSensor_Model 20150

// Initialisation
DualVNH5019MotorShield md;
// Version 1
//SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4810.44, 9.269, -40.303);
//SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 11680.9, 14.505, 121.366);
//SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5482.45, 9.396, -19.069);
// Version 2
//SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4676.8, 9.444, -50.382); // B = 9.444
//SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 7844.25, 10.574, 32.843); // B = 9.574
//SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5460.31, 10.052, -29.841); // B = 8.852
// Version 3
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4620.2, 8.10397, -36.7569);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 11750.6, 14.7919, 136.813);
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5340.28, 9.47195, -11.1088);
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
float kpLeftME = 2.72;
float kiLeftME = 0.2;
float kdLeftME = 0.1;
float kpRightME = 3.20; //3.12
float kiRightME = 0.2;
float kdRightME = 0.1;
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
int setpoint = 80;
// Robot Movement
unsigned int oneGridDistance = 10800; // 10980
unsigned int turnGridDistance = 14285;
unsigned int distance = 0;
unsigned int totalDistance = 0;
// Sensors
String sensorOutput = "";
String debugOutput = "";
int counter = 0;
int sensorSampleSize = 25;
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
float angleCalibrationThreshold = 0;
float calibrationError = 0;
// Others
String cc = "";
boolean explorationFlag = false;
boolean fastestPathFlag = false;
// Testing Parameters (To Remove)
RunningMedian analogReadings = RunningMedian(sensorSampleSize);
RunningMedian analogReadings2 = RunningMedian(sensorSampleSize);
RunningMedian analogReadings3 = RunningMedian(sensorSampleSize);
boolean rightFacing = false;
boolean leftFacing = false;

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
  restartPID();
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag) {
    if (Serial.available() > 0) {
      cc = char(Serial.read());
      if (cc == "U") {
        goStraightNGrids(1);
        delay(125);
//        calibrateLeftDistance();
//        delay(125);
//        calibrateLeftAngle();
//        delay(125);
        getSensorsDistanceRM(sensorSampleSize);
        printPIDRPM();
        restartPID();        
        delay(125);
      } else if (cc == "L") {
        turnLeftOneGrid();
        delay(125);
//        calibrateLeftDistance();
//        delay(125);
//        calibrateLeftAngle();
//        delay(125);
        getSensorsDistanceRM(sensorSampleSize);
        printPIDRPM();
        restartPID();
        delay(125);
      } else if (cc == "R") {
        turnRightOneGrid();
        delay(125);
//        calibrateLeftDistance();
//        delay(125);
//        calibrateLeftAngle();
//        delay(125);
        getSensorsDistanceRM(sensorSampleSize);
        printPIDRPM();
        restartPID();
        delay(125);
      } else if (cc == "D") {
        turnRightOneGrid();
        printPIDRPM();
        restartPID();
        delay(125);
        turnRightOneGrid();
        delay(125);
//        calibrateLeftDistance();
//        delay(125);
//        calibrateLeftAngle();
//        delay(125);
        getSensorsDistanceRM(sensorSampleSize);
        printPIDRPM();
        restartPID();
        delay(125);
      } else if (cc == "Z") {
        getSensorsDistanceRM(sensorSampleSize);
      } else if (cc == "#") {
        explorationFlag = false;
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
  if (PIDOutputLeftME > 4 || PIDOutputRightME > 4) {
    restartPID();
  }
}

void restartPID() {
  RPMLeft = RPMRight = 0;
  PIDOutputLeftME = PIDOutputRightME = 0;
  previousPIDOutputLeftME = previousPIDOutputRightME = 0;
  errorLeftME = errorRightME = 0;
  previousErrorLeftME = previousErrorRightME = 0;
  previousPreviousErrorLeftME = previousPreviousErrorRightME = 0;
}

void printPIDRPM(){
  debugOutput = "d";
  debugOutput = debugOutput + "LR_PID: " + previousPIDOutputLeftME + " " + previousPIDOutputRightME + "|" + "LR_RPM: " + previousPIDOutputLeftME + " " + previousPIDOutputRightME;
  Serial.println(debugOutput);
}
// ==================== Robot Movement ====================

void goStraightNGrids(int n)
{
  distance = n * oneGridDistance;
  totalDistance = 0;
  while (1) {
    if (totalDistance >= distance) {
      md.setBrakes(400, 400);
      break;
    } else {
      moveForward();
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void moveForward()
{
  PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
  md.setSpeeds(PIDOutputRightME * 150, PIDOutputLeftME * 150);
  delayMicroseconds(4450);
}

void turnLeftOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance > turnGridDistance) {
      md.setBrakes(400, 400);
      break;
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(PIDOutputRightME * 150, -PIDOutputLeftME * 150);
      delayMicroseconds(4700);
      totalDistance = totalDistance + RPMLeft + RPMRight; 
    }
  }
}

void turnRightOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance > turnGridDistance) {
      md.setBrakes(400, 400);
      break;
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(-PIDOutputRightME * 150, PIDOutputLeftME * 150);
      delayMicroseconds(4700);
      totalDistance = totalDistance + RPMLeft + RPMRight; 
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
    //analogReadings.add(analogRead(srSensorLeft1));
    //analogReadings.add(analogRead(srSensorLeft2));
    //analogReadings.add(analogRead(lrSensorRight1));
  }
  Serial.println("===== Median ====");
  Serial.println(analogReadings.getMedian());
  Serial.println(analogReadings2.getMedian());
  Serial.println(analogReadings3.getMedian());
  Serial.println("===== Average =====");
  Serial.println(analogReadings.getAverage(5));
  Serial.println(analogReadings2.getAverage(5));
  Serial.println(analogReadings3.getAverage(5));
}

void getSensorsDistanceRM(int n)
{
  sensorOutput = "r";
  debugOutput = "d";
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
  // Short Range Sensor Front 1
  srSensorFront1Distance = round(srSensorFront1DistanceRM.getAverage(5));
  if (0 <= srSensorFront1Distance && srSensorFront1Distance <= 10) {
    srSensorFront1DistanceOutput = -1;
  } else if (11 <= srSensorFront1Distance && srSensorFront1Distance <= 17) {
    srSensorFront1DistanceOutput = 1;
  } else if (20 <= srSensorFront1Distance && srSensorFront1Distance <= 25) {
    srSensorFront1DistanceOutput = 2;
  } else {
    srSensorFront1DistanceOutput = 0;
  }
  // Short Range Sensor Front 2
  srSensorFront2Distance = round(srSensorFront2DistanceRM.getAverage(5));
  if (0 <= srSensorFront2Distance && srSensorFront2Distance <= 10) {
    srSensorFront2DistanceOutput = -1;
  } else if (11 <= srSensorFront2Distance && srSensorFront2Distance <= 17) {
    srSensorFront2DistanceOutput = 1;
  } else if (19 <= srSensorFront2Distance && srSensorFront2Distance <= 25) {
    srSensorFront2DistanceOutput = 2;
  } else {
    srSensorFront2DistanceOutput = 0;
  }
  // Short Range Sensor Front 3
  srSensorFront3Distance = round(srSensorFront3DistanceRM.getAverage(5));
  if (0 <= srSensorFront3Distance && srSensorFront3Distance <= 10) {
    srSensorFront3DistanceOutput = -1;
  } else if (11 <= srSensorFront3Distance && srSensorFront3Distance <= 18) {
    srSensorFront3DistanceOutput = 1;
  } else if (21 <= srSensorFront3Distance && srSensorFront3Distance <= 25) {
    srSensorFront3DistanceOutput = 2;
  } else {
    srSensorFront3DistanceOutput = 0;
  }
  // Short Range Sensor Left 1
  srSensorLeft1Distance = srSensorLeft1DistanceRM.getMedian();
  Serial.println(srSensorLeft1Distance);
  if (0 <= srSensorLeft1Distance && srSensorLeft1Distance <= 10) {
    srSensorLeft1DistanceOutput = -1;
  } else if (11 <= srSensorLeft1Distance && srSensorLeft1Distance <= 18) {
    srSensorLeft1DistanceOutput = 1;
  } else if (21 <= srSensorLeft1Distance && srSensorLeft1Distance <= 25) {
    srSensorLeft1DistanceOutput = 2;
  } else {
    srSensorLeft1DistanceOutput = 0;
  }
  // Short Range Sensor Left 2
  srSensorLeft2Distance = srSensorLeft2DistanceRM.getMedian();
  if (0 <= srSensorLeft2Distance && srSensorLeft2Distance <= 10) {
    srSensorLeft2DistanceOutput = -1;
  } else if (11 <= srSensorLeft2Distance && srSensorLeft2Distance <= 18) {
    srSensorLeft2DistanceOutput = 1;
  } else if (21 <= srSensorLeft2Distance && srSensorLeft2Distance <= 25) {
    srSensorLeft2DistanceOutput = 2;
  } else {
    srSensorLeft2DistanceOutput = 0;
  }
  // Long Range Sensor Right 1
  lrSensorRight1Distance = lrSensorRight1DistanceRM.getMedian();
  if (0 <= lrSensorRight1Distance && lrSensorRight1Distance <= 10) {
    lrSensorRight1DistanceOutput = -1;
  } else if (11 <= lrSensorRight1Distance && lrSensorRight1Distance <= 18) {
    lrSensorRight1DistanceOutput = 1;
  } else if (21 <= lrSensorRight1Distance && lrSensorRight1Distance <= 25) {
    lrSensorRight1DistanceOutput = 2;
  } else {
    lrSensorRight1DistanceOutput = 0;
  }
  debugOutput = debugOutput + srSensorFront1Distance + "|" + srSensorFront2Distance + "|" + srSensorFront3Distance + "|" + srSensorLeft1Distance + "|" + srSensorLeft2Distance + "|" + lrSensorRight1Distance;
  sensorOutput = sensorOutput + srSensorFront1DistanceOutput + "|" + srSensorFront2DistanceOutput + "|" + srSensorFront3DistanceOutput + "|" + srSensorLeft1DistanceOutput + "|" + srSensorLeft2DistanceOutput + "|" + lrSensorRight1DistanceOutput;
  Serial.println(sensorOutput);
  Serial.println(debugOutput);
}

// ==================== Calibration ====================

void calibrateTurnRight(double n)
{
  distance = turnGridDistance * n;
  totalDistance = 0;
  while (1) {
    if (totalDistance >= distance) {
      md.setBrakes(-375, 375);
      break;
    } else {
      md.setSpeeds(-75, 75);
      delayMicroseconds(4230);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void calibrateTurnLeft(double n)
{
  distance = turnGridDistance * n;
  totalDistance = 0;
  while (1) {
    if (totalDistance >= distance) {
      md.setBrakes(375, -375);
      break;
    } else {
      md.setSpeeds(75, -75);
      delayMicroseconds(4230);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void calibrateLeftAngle()
{
  calibrationError = SRSensorLeft1.distance() - SRSensorLeft2.distance();
  if (SRSensorLeft1.distance() < 12 && SRSensorLeft2.distance() < 12) {
    if (calibrationError > 1 && calibrationError < 9) {
      while (calibrationError > 4) {
        calibrateTurnLeft(0.05);
        calibrationError = SRSensorLeft1.distance() - SRSensorLeft2.distance();
        delayMicroseconds(5000);
      }
    } else if (calibrationError < -1 && calibrationError > -9) {
      while (calibrationError < -4) {
        calibrateTurnRight(0.05);
        calibrationError = SRSensorLeft1.distance() - SRSensorLeft2.distance();
        delayMicroseconds(5000);
      }
    }
  }
}

void calibrateLeftDistance()
{
  if (SRSensorLeft1.distance() < 5 && SRSensorLeft2.distance() < 5) {
    restartPID();
    turnLeftOneGrid();
    delayMicroseconds(5000);
    while (SRSensorFront1.distance() < 6 && SRSensorFront2.distance() < 6 && SRSensorFront2.distance() < 6) {
      restartPID();
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(-PIDOutputRightME * 50, -PIDOutputLeftME * 50);
      delayMicroseconds(5000);
    }
    delayMicroseconds(5000);
    restartPID();
    turnRightOneGrid();
  }
  if (SRSensorLeft1.distance() > 7 && SRSensorLeft2.distance() > 7) {
    restartPID();
    turnLeftOneGrid();
    delayMicroseconds(5000);
    while (SRSensorFront1.distance() > 7 && SRSensorFront2.distance() < 7 && SRSensorFront2.distance() < 7) {
      restartPID();
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(PIDOutputRightME * 50, PIDOutputLeftME * 50);
      delayMicroseconds(5000);
    }
    delayMicroseconds(5000);
    restartPID();
    turnRightOneGrid();
  }
}
