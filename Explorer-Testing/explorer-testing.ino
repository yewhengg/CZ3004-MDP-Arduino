#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "PinChangeInt.h"
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
// Tuning parameters for week 9
// PID
float kpLeftME = 2.72;
// JERKY?
float kiLeftME = 0.2;
float kdLeftME = 0.1;
float kpRightME = 3.20; // Original = 3.12
// JERKY?
float kiRightME = 0.2;
float kdRightME = 0.1;

// Robot Movement
unsigned int oneGridDistance = 10100; // Original = 10800
unsigned int turnGridDistance = 14285;
// auto-cali parameters
float MIN_DISTANCE_CALIBRATE = 12;   // distance away from obstacle to trigger calibration
float ANGLE_CALIBRATION_THRESHOLD = 1.0;  // error within this value not trigger calibration
float LEFT_ANGLE_CALIBRATION_THRESHOLD = 0.5;  // error within this value not trigger calibration
float FRONT_SENSORS_DISTANCE_THRESHOLD[2] = {4.5, 6.75};
float LEFT_SENSORS_DISTANCE_THRESHOLD[2] = {4.5, 5.4}; 
// delay
int moveForwardDelay = 5; // Original = 4450
int turnDelay = 5;  // Original = 4700
int explorationDelay = 125; // Original = 125, try 2.5
int calibrationDelay = 10; // Not used

// COUNTER
int COUNT = 3;  //counter the num of forward movement


// Initialisation
DualVNH5019MotorShield md;
// Old
//SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4620.2, 8.10397, -36.7569);
//SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 11750.6, 14.7919, 136.813);
//SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5340.28, 9.47195, -11.1088);
//SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 6852.53, 9.900, 16.1893);
//SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 8273.7, 11.014, 63.616);
//SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 24845.1, 21.719, 212.088);
// New
SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 3509.11, 10.1626, -83.674);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 5305.81, 11.1486, -34.9659);
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5122.21, 12.9189, -31.1762);
SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 5177.84, 11.0427, -21.9181);
SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 7840.02, 14.9919, 43.0052);
SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 302904, 169.317, 1249.89);
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
int setpoint = 80;
unsigned int distanceToMove = 0;
unsigned int totalDistance = 0;
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
float SRFRONT_1_RANGE[3] = {12.42, 27.45, 40.95};
float SRFRONT_2_RANGE[3] = {12.21, 24.02, 35.54};
float SRFRONT_3_RANGE[3] = {12.45, 23.70, 37.32};
float SRLEFT_1_RANGE[3] = {9.33, 18.80, 27.32};
float SRLEFT_2_RANGE[3] = {9.31, 17.94, 25.25};
float LRRIGHT_1_RANGE[3] = {10.11, 20.55, 29.54};
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
float frontSensor3ToWall = 0;
float leftSensor1ToWall = 0;
float leftSensor2ToWall = 0;
float calibrationAngleError = 0;


// Others
String sRead = "";
String cc = "";
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
  if (Serial.available() > 0) {
      explorationFlag = true;
      exploration();
  }
}

// ==================== Modes ====================

void exploration()
{
  restartPID();
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag) {
    if (Serial.available() > 0) {
      if(COUNT==1){
        calibrateLeftDistance();
        COUNT=3;
      }
      sRead = Serial.readString();
      cc = sRead.substring(0,2);
      sRead = sRead.substring(2);
       if(cc == "11"){
        MIN_DISTANCE_CALIBRATE = sRead.toFloat();
      } else if(cc == "12"){
        ANGLE_CALIBRATION_THRESHOLD = sRead.toFloat();
      } else if(cc == "13"){
        LEFT_ANGLE_CALIBRATION_THRESHOLD = sRead.toFloat();
      } else if(cc == "14"){
        kpLeftME = sRead.toFloat();
      } else if(cc == "15"){
        kiLeftME = sRead.toFloat();
      } else if(cc == "16"){
        kdLeftME = sRead.toFloat();
      } else if(cc == "17"){
        kpRightME = sRead.toFloat();
      }else if(cc == "18"){
        kiRightME = sRead.toFloat();
      }else if(cc == "19"){
        kdRightME = sRead.toFloat();
      }else if(cc == "20"){
        oneGridDistance = sRead.toInt();
      }else if(cc == "21"){
        turnGridDistance = sRead.toInt();
      }else if(cc == "22"){
        moveForwardDelay = sRead.toInt();
      }else if(cc == "23"){
        turnDelay = sRead.toInt();
      }else if(cc == "24"){
        explorationDelay = sRead.toInt();
      }else if(cc == "25"){
        calibrationDelay = sRead.toInt();
      }else if(cc == "26"){
        FRONT_SENSORS_DISTANCE_THRESHOLD[0] = sRead.toFloat();
      }else if(cc == "27"){
        FRONT_SENSORS_DISTANCE_THRESHOLD[1] = sRead.toFloat();
      }else if(cc == "28"){
        LEFT_SENSORS_DISTANCE_THRESHOLD[0] = sRead.toFloat();
      }else if(cc == "29"){
        LEFT_SENSORS_DISTANCE_THRESHOLD[1] = sRead.toFloat();
      }else if(cc == "30"){
        COUNT = sRead.toInt();
      }
      else if (cc == "UU") {
        COUNT--;
        goStraightNGrids(1);
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        debugSensorDistance();
        //debugPID();
        //debugDelay();
        restartPID();
      } else if (cc == "LL") {
        turnLeftOneGrid();
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        debugSensorDistance();
        // print pid value
        //debugPID();
        //print delay time
        //debugDelay();
        restartPID();
      } else if (cc == "RR") {
        turnRightOneGrid();
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        debugSensorDistance();
       // debugPID();
        //debugDelay();
        restartPID();
      } else if (cc == "DD") {
        turnRightOneGrid();
        delay(explorationDelay);
        restartPID();
        turnRightOneGrid();
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        restartPID();
      } else if (cc == "ZZ") {
        getSensorsVoltageRM(sensorSampleSize);
        getSensorsDistanceRM(sensorSampleSize);
        delay(explorationDelay);
        debugSensorDistance();
      } else if (cc == "##") {
        explorationFlag = false;
        delay(explorationDelay);
        return;
      }
      // ===== Debug (To Remove) ===== 
      else if (cc == "CC") {
        calibrate();
        // calibrateLeftWall();
      }
      else if (cc == "AC") {
        calibrateFrontAngle();
        // calibrateLeftWall();
      }else if (cc == "DC") {
         calibrateFrontDistance(0);
      }else if(cc == "LC"){
        calibrateLeftDistance();
      }else if(cc == "CL"){
        calibrateTurnLeftOneGrid();
      }
      else if(cc == "CR"){
        calibrateTurnRightOneGrid();
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

// ==================== Robot Movement ====================

void goStraightNGrids(int n)
{
  distanceToMove = n * oneGridDistance;
  totalDistance = 0;
  while (1) {
    if (totalDistance >= distanceToMove) {
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
  // delayMicroseconds(moveForwardDelay);
  delay(moveForwardDelay);
}

void turnLeftOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance >= turnGridDistance) {
      md.setBrakes(400, 400);
      break;
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(PIDOutputRightME * 150, -PIDOutputLeftME * 150);
      delay(turnDelay);
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
      delay(turnDelay);
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
//    analogReadings.add(analogRead(srSensorFront1));
//    analogReadings2.add(analogRead(srSensorFront2));
//    analogReadings3.add(analogRead(srSensorFront3));
//    analogReadings.add(analogRead(srSensorLeft1));
//    analogReadings2.add(analogRead(srSensorLeft2));
      analogReadings.add(analogRead(lrSensorRight1));
  }
//  Serial.println("===== Median ====");
//  Serial.println(analogReadings.getMedian());
//  Serial.println(analogReadings2.getMedian());
//  Serial.println(analogReadings3.getMedian());
  Serial.println("===== Average =====");
  Serial.println(analogReadings.getAverage(5));
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
  for (counter = 0; counter < n; counter++) {
    srSensorFront1DistanceRM.add(SRSensorFront1.distance());
    srSensorFront2DistanceRM.add(SRSensorFront2.distance());
    srSensorFront3DistanceRM.add(SRSensorFront3.distance());
    srSensorLeft1DistanceRM.add(SRSensorLeft1.distance());
    srSensorLeft2DistanceRM.add(SRSensorLeft2.distance());
    lrSensorRight1DistanceRM.add(LRSensorRight1.distance());
  }
  // Short Range Sensor Front 1
  srSensorFront1Distance = srSensorFront1DistanceRM.getAverage(5);
  if (srSensorFront1Distance <= SRFRONT_1_RANGE[0]) {
    srSensorFront1DistanceOutput = -1;
  } else if (srSensorFront1Distance <= SRFRONT_1_RANGE[1]) {
    srSensorFront1DistanceOutput = 1;
  } else if (srSensorFront1Distance <= SRFRONT_1_RANGE[2]) {
    srSensorFront1DistanceOutput = 2;
  } else {
    srSensorFront1DistanceOutput = 0;
  }
  // Short Range Sensor Front 2
  srSensorFront2Distance = srSensorFront2DistanceRM.getAverage(5);
  if (srSensorFront2Distance <= SRFRONT_2_RANGE[0]) {
    srSensorFront2DistanceOutput = -1;
  } else if (srSensorFront2Distance <= SRFRONT_2_RANGE[1]) {
    srSensorFront2DistanceOutput = 1;
  } else if (srSensorFront2Distance <= SRFRONT_2_RANGE[2]) {
    srSensorFront2DistanceOutput = 2;
  } else {
    srSensorFront2DistanceOutput = 0;
  }
  // Short Range Sensor Front 3
  srSensorFront3Distance = srSensorFront3DistanceRM.getAverage(5);
  if (srSensorFront3Distance <= SRFRONT_3_RANGE[0]) {
    srSensorFront3DistanceOutput = -1;
  } else if (srSensorFront3Distance <= SRFRONT_3_RANGE[1]) {
    srSensorFront3DistanceOutput = 1;
  } else if (srSensorFront3Distance <= SRFRONT_3_RANGE[2]) {
    srSensorFront3DistanceOutput = 2;
  } else {
    srSensorFront3DistanceOutput = 0;
  }
  // Short Range Sensor Left 1
  srSensorLeft1Distance = srSensorLeft1DistanceRM.getAverage(5);
  if (srSensorLeft1Distance <= SRLEFT_1_RANGE[0]) {
    srSensorLeft1DistanceOutput = -1;
  } else if (srSensorLeft1Distance <= SRLEFT_1_RANGE[1]) {
    srSensorLeft1DistanceOutput = 1;
  } else if (srSensorLeft1Distance <= SRLEFT_1_RANGE[2]) {
    srSensorLeft1DistanceOutput = 2;
  } else {
    srSensorLeft1DistanceOutput = 0;
  }
  // Short Range Sensor Left 2
  srSensorLeft2Distance = srSensorLeft2DistanceRM.getAverage(5);
  if (srSensorLeft2Distance <= SRLEFT_2_RANGE[0]) {
    srSensorLeft2DistanceOutput = -1;
  } else if (srSensorLeft2Distance <= SRLEFT_2_RANGE[1]) {
    srSensorLeft2DistanceOutput = 1;
  } else if (srSensorLeft2Distance <= SRLEFT_2_RANGE[2]) {
    srSensorLeft2DistanceOutput = 2;
  } else {
    srSensorLeft2DistanceOutput = 0;
  }
  // Long Range Sensor Right 1
  lrSensorRight1Distance = lrSensorRight1DistanceRM.getAverage(5);
  if (lrSensorRight1Distance <= LRRIGHT_1_RANGE[0]) {
    lrSensorRight1DistanceOutput = -1;
  } else if (lrSensorRight1Distance <= LRRIGHT_1_RANGE[1]) {
    lrSensorRight1DistanceOutput = 1;
  } else if (lrSensorRight1Distance <= LRRIGHT_1_RANGE[2]) {
    lrSensorRight1DistanceOutput = 2;
  } else {
    lrSensorRight1DistanceOutput = 0;
  }
  sensorOutput = sensorOutput + srSensorFront1DistanceOutput + "|" + srSensorFront2DistanceOutput + "|" + srSensorFront3DistanceOutput + "|" + srSensorLeft1DistanceOutput + "|" + srSensorLeft2DistanceOutput + "|" + lrSensorRight1DistanceOutput;
  Serial.println(sensorOutput);
}


// ==================== Calibration ====================


void calibrate() {
  float frontSensor3ToWall = SRSensorFront3.distance();  //getDistance(1);
  float frontSensor1ToWall = SRSensorFront1.distance(); //getDistance(3);
  if(frontSensor3ToWall > MIN_DISTANCE_CALIBRATE + 10 || frontSensor1ToWall > MIN_DISTANCE_CALIBRATE + 10) return;
  delay(calibrationDelay);
  calibrateFrontAngle();
  delay(calibrationDelay);
  calibrateFrontDistance(0);
  delay(calibrationDelay);
  calibrateFrontAngle();
}


void calibrateFrontDistance(float offset)
{
  frontSensor1ToWall = 0;
  frontSensor3ToWall = 0;
  float error;
  while(1) {
    //Original
    frontSensor1ToWall = SRSensorFront1.distance() - offset;
    frontSensor3ToWall = SRSensorFront3.distance() - offset;
    // 5cm is the target of the distance between obstacle/wall and robot.
    error = 5.0 - ((frontSensor1ToWall + frontSensor3ToWall) / 2);

    if(frontSensor1ToWall > MIN_DISTANCE_CALIBRATE || 
       frontSensor3ToWall > MIN_DISTANCE_CALIBRATE) {
      break;
     }

    //abort if within margin of error
    if ((frontSensor3ToWall >= FRONT_SENSORS_DISTANCE_THRESHOLD[0] && frontSensor3ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[1]) || 
        (frontSensor1ToWall >= FRONT_SENSORS_DISTANCE_THRESHOLD[0] && frontSensor1ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[1])) {
      break;
     }

    if (frontSensor1ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[0] || 
        frontSensor3ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[0]) {
      //reverse normally
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 30);
      md.setSpeeds(PIDOutputRightME * -100, PIDOutputLeftME * -100);
      delay(abs(error) * 75);  //100
      md.setBrakes(400, 400);
    }
    else if(frontSensor1ToWall > FRONT_SENSORS_DISTANCE_THRESHOLD[1] || 
        frontSensor3ToWall > FRONT_SENSORS_DISTANCE_THRESHOLD[1]) {  
      //move forward
      //moveForward(100,0.9);
      // Serial.println("Moving forwards");
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 30);
      md.setSpeeds(PIDOutputRightME * 100, PIDOutputLeftME * 100);
      delay(abs(error) * 75);    //100
      md.setBrakes(400, 400);
    }
  }
}

 void calibrateFrontSensorsAngle(float error)
{
  if (error > ANGLE_CALIBRATION_THRESHOLD) {
    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    md.setSpeeds(PIDOutputRightME * 50, -PIDOutputLeftME * 50);
    delay(abs(error * 50));
    md.setBrakes(400, 400);
  } else if (error < -ANGLE_CALIBRATION_THRESHOLD) {
    PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
    md.setSpeeds(-PIDOutputRightME * 50, PIDOutputLeftME * 50);
    delay(abs(error * 50));
    md.setBrakes(400, 400);
  }
}

void calibrateFrontAngle()
{
  int counter = 0;
  frontSensor1ToWall = 0;
  frontSensor3ToWall = 0;
  float calibrationAngleError = 0;
  double angleDist = 0;
  while(counter < 15){
    frontSensor1ToWall = SRSensorFront1.distance();
    frontSensor3ToWall = SRSensorFront3.distance();
    calibrationAngleError = frontSensor1ToWall - frontSensor3ToWall;
    counter++;
    if(abs(calibrationAngleError) <= ANGLE_CALIBRATION_THRESHOLD){
      break;
    }
    calibrateFrontSensorsAngle(calibrationAngleError);
    md.setBrakes(400,400);
  }
}


void calibrateLeftDistance()
{
  // Function that implement the logic when to turn left 90degree, and then call calibrateFrontDistance. 
  leftSensor1ToWall = 0;
  leftSensor2ToWall = 0;
  leftSensor1ToWall = SRSensorLeft1.distance();
  leftSensor2ToWall = SRSensorLeft2.distance();
  float diff = abs(leftSensor1ToWall - leftSensor2ToWall);
  if(leftSensor1ToWall > MIN_DISTANCE_CALIBRATE ||  leftSensor2ToWall > MIN_DISTANCE_CALIBRATE){
    return;
  }

//abort if within margin of error
  if (diff < LEFT_ANGLE_CALIBRATION_THRESHOLD && ((leftSensor1ToWall >= LEFT_SENSORS_DISTANCE_THRESHOLD[0] && leftSensor1ToWall < LEFT_SENSORS_DISTANCE_THRESHOLD[1]) || (leftSensor2ToWall >= LEFT_SENSORS_DISTANCE_THRESHOLD[0] && leftSensor2ToWall < LEFT_SENSORS_DISTANCE_THRESHOLD[1]))) {
    return;
    }
  if (diff >= LEFT_ANGLE_CALIBRATION_THRESHOLD || (leftSensor1ToWall < LEFT_SENSORS_DISTANCE_THRESHOLD[0] && leftSensor2ToWall < LEFT_SENSORS_DISTANCE_THRESHOLD[0]) || (leftSensor1ToWall > LEFT_SENSORS_DISTANCE_THRESHOLD[1] && leftSensor2ToWall > LEFT_SENSORS_DISTANCE_THRESHOLD[1])) 
  {
    calibrateTurnLeftOneGrid();
    calibrateFrontDistance(1.5);
    delay(calibrationDelay);
    calibrateFrontAngle();
    delay(calibrationDelay); 
    calibrateTurnRightOneGrid();
  }
}

void calibrateTurnLeftOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance >= turnGridDistance + 800) {
      md.setBrakes(400, 400);
      break;
    } else {
//      md.setSpeeds(80, -80);
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 50);
      md.setSpeeds(PIDOutputRightME * 100, -PIDOutputLeftME * 100);
      delay(turnDelay);
      totalDistance = totalDistance + RPMLeft + RPMRight; 
    }
  }
}

void calibrateTurnRightOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance > turnGridDistance + 800) {
      md.setBrakes(400, 400);
      break;
    } else {
//      md.setSpeeds(-80, 80);
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, 50);
      md.setSpeeds(-PIDOutputRightME * 100, PIDOutputLeftME * 100);
      delay(turnDelay);
      totalDistance = totalDistance + RPMLeft + RPMRight; 
    }
  }
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
  Serial.println(debugOutput);
}

void debugDelay()
{
  debugOutput = "d";
  debugOutput = debugOutput + "moveForwardDelay = " + moveForwardDelay + " | turnDelay = " + turnDelay + " | explorationDelay = " + explorationDelay + " | calibrationDelay = " + calibrationDelay;
  Serial.println(debugOutput);
}
