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


// Robot Movement
unsigned int oneGridDistance = 2900;
unsigned int turnGridDistance = 16000;  //ori=14600
unsigned int turnRightDistance = 16100; //ori=14600
unsigned int moveSpeed = 300;
unsigned int turnSpeed = 250;
float soffset = 8.10;

// Auto-Cali Parameters
float LEFT_ANGLE_OFFSET = -0.7;       // error within this value not trigger calibration
float LEFT_DIS_OFFSET = 0.0; 
// float MIN_DISTANCE_CALIBRATE = 12;             // distance away from obstacle to trigger calibration
float ANGLE_CALIBRATION_THRESHOLD = 1.0;       // error within this value not trigger calibration 
// float LEFT_ANGLE_CALIBRATION_THRESHOLD = 0.10; // error within this value not trigger calibration
float FRONT_SENSORS_DISTANCE_THRESHOLD[2] = { 6.0 / 2, 7.0 / 2};  // distance away from obstacle after cali
// float LEFT_SENSORS_DISTANCE_THRESHOLD[2] = {1.0, 3.0}; //{2.5/2, 3.5/2};
// Delay
int moveForwardDelay = 15; // Original = 4450
int turnDelay = 5;         // Original = 4700
int explorationDelay = 250;  // Original = 125, try 2.5
int calibrationDelay = 10; // Not used

// Sensor Range
float SRFRONT_1_RANGE[3] = {11.10, 19.82, 37.55}; // {11.10, 22.10, 37.55}; 
float SRFRONT_2_RANGE[3] = {11.50, 20.90, 36.02};               //{11.10, 22.20, 36.02}; 
float SRFRONT_3_RANGE[3] = {10.60, 20.20, 40.75};               //{11.10, 23.10, 40.75};
float SRLEFT_1_RANGE[3] = {11.52, 22.30, 34.17};                //{13.63, 20.60, 27.52}
float SRLEFT_2_RANGE[3] = {11.70, 23.20, 32.16};                //{13.63, 20.60, 26.45}
float LRRIGHT_1_RANGE[5] = {12.52, 22.70, 31.00, 44.04, 51.21}; //{13.90, 20.55, 29.54}

// Initialisation
DualVNH5019MotorShield md;

SharpIR SRSensorFront1(srSensorFront1, SRSensor_Model, 4237.64, -49.8888, 8.66356);
SharpIR SRSensorFront2(srSensorFront2, SRSensor_Model, 6313.93, 16.0381, 9.38024);
SharpIR SRSensorFront3(srSensorFront3, SRSensor_Model, 5170.73, -15.4715, 10.4833);
SharpIR SRSensorLeft1(srSensorLeft1, SRSensor_Model, 5285.29, -14.8405,8.24699 );
SharpIR SRSensorLeft2(srSensorLeft2, SRSensor_Model, 5440.65, -2.86093, 7.79677 );
SharpIR LRSensorRight1(lrSensorRight1, LRSensor_Model, 94637.7, 541.154, 83.5766); 

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
  getSensorsDistanceRM(sensorSampleSize);
  while (explorationFlag)
  {
    if (Serial.available() > 0)
    {
      sRead = char(Serial.read());
     // Algo Commands
      if (sRead == "U")
      {
        goStraightNGrids(1);
        getSensorsDistanceRM(sensorSampleSize);
        // to make sure robot is straight
//        checktAngle(LEFT_ANGLE_OFFSET);
      } 
      else if (sRead == "2")
      {
        goStraightNGrids(2);
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (sRead == "L")
      {
        delay(explorationDelay);
        turnLeftOneGrid();
        // delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (sRead == "R")
      {
        delay(explorationDelay); 
        turnRightOneGrid();
        // delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (sRead == "D")
      {
        turnRightOneGrid();
        // delay(explorationDelay);
        turnRightOneGrid();
        // delay(explorationDelay);
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (sRead == "Z")
      {
        getSensorsDistanceRM(sensorSampleSize);
        debugSensorDistance();
      }
      // Send when sensors 1 - 3 all -1
      else if (sRead == "C")
      {
        calibrate_FRONT();
        getSensorsDistanceRM(sensorSampleSize);
      }
      // Send when sensors 1 - 5 all -1
      else if (sRead == "F")
      {
        calibrate_FULL();
        getSensorsDistanceRM(sensorSampleSize);
      }
      else if (sRead == "#")
      {
        explorationFlag = false;
        return;
      }
    }
  }
}


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
      delay(turnDelay);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

// ==================== Sensors ====================

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

void calibrate_FRONT()
{
  float frontSensor2ToWall = SRSensorFront2.distance(); //getDistance(2);
  // frontSensor2ToWall should be slightly smaller than the SRFRONT_2_RANGE[1]
  if (frontSensor2ToWall > SRFRONT_2_RANGE[1] - 1)
    return;
  
  // Front angle -> Front dis -> Front angle
  calibrateAngle(0);
  delay(calibrationDelay);
  calibrateDistance(0);
  delay(calibrationDelay);
  calibrateAngle(0);
}

void calibrate_FULL()
{
  float frontSensor2ToWall = SRSensorFront2.distance(); //getDistance(2);
  // frontSensor2ToWall should be slightly smaller than the SRFRONT_2_RANGE[1]
  if (frontSensor2ToWall > SRFRONT_2_RANGE[1] - 1)
    return;
  // Front angle - Turn left -> Front angle -> Front dis -> 
  // Turn right -> Front angle -> Front dis
  calibrateAngle(0);
  delay(calibrationDelay);
  turnLeftOneGrid();
  delay(calibrationDelay);
  calibrateAngle(LEFT_ANGLE_OFFSET);
  delay(calibrationDelay);
  calibrateDistance(LEFT_DIS_OFFSET);
  delay(calibrationDelay);
  turnRightOneGrid();
  delay(calibrationDelay);
  calibrateAngle(0);
  delay(calibrationDelay);
  calibrateDistance(0);
}

void calibrateSensorsAngle(float error, float offset)
{
  if (error > (ANGLE_CALIBRATION_THRESHOLD + offset))
  {
    md.setSpeeds(100, -100);
    delay(abs(error * 25));
    md.setBrakes(400, 400);
  }
  else if (error < -(ANGLE_CALIBRATION_THRESHOLD + offset))
  {
    md.setSpeeds(-100, 100);
    delay(abs(error * 25));
    md.setBrakes(400, 400);
  }
}

void calibrateAngle(float offset)
{
  int counter = 0;
  frontSensor1ToWall = 0;
  frontSensor3ToWall = 0;
  float calibrationAngleError = 0;
  while (counter < 15)
  {
    frontSensor1ToWall = SRSensorFront1.distance();
    frontSensor3ToWall = SRSensorFront3.distance();
    calibrationAngleError = frontSensor1ToWall - frontSensor3ToWall;
    counter++;
    if (abs(calibrationAngleError) <= (ANGLE_CALIBRATION_THRESHOLD + offset))
    {
      break;
    }
    calibrateSensorsAngle(calibrationAngleError, offset);
    md.setBrakes(400, 400);
  }
}

void calibrateDistance(float offset)
{
  frontSensor2ToWall = 0;
  float error;
  while (1)
  {
    //Original
    frontSensor2ToWall = SRSensorFront2.distance();
    error = 3.5 - frontSensor2ToWall;

    // if(frontSensor2ToWall > MIN_DISTANCE_CALIBRATE)
    //   break;

    // offset: used when front wall and left wall distance is diff
    if (frontSensor2ToWall >= (FRONT_SENSORS_DISTANCE_THRESHOLD[0] + offset) 
            && frontSensor2ToWall < (FRONT_SENSORS_DISTANCE_THRESHOLD[1] + offset) )
      break;

    if (frontSensor2ToWall < FRONT_SENSORS_DISTANCE_THRESHOLD[0])
    {
      md.setSpeeds(-120, -120);
      delay(abs(error) * 50); //100
      md.setBrakes(400, 400);
    }

    else if (frontSensor2ToWall > FRONT_SENSORS_DISTANCE_THRESHOLD[1])
    {
      md.setSpeeds(120, 120);
      delay(abs(error) * 50); //100
      md.setBrakes(400, 400);
    }
  }
}

// run in each U
void checktAngle(float offset)
{
  int counter = 0;
  leftSensor1ToWall = 0;
  leftSensor2ToWall = 0;
  float calibrationAngleError = 0;
  while (counter < 5)
  {
    leftSensor1ToWall = SRSensorLeft1.distance();
    leftSensor2ToWall = SRSensorLeft2.distance();
    calibrationAngleError = leftSensor1ToWall - leftSensor2ToWall;
    counter++;
    if (abs(calibrationAngleError) <= (ANGLE_CALIBRATION_THRESHOLD + offset))
    {
      break;
    }
    calibrateSensorsAngle(calibrationAngleError, offset);
    md.setBrakes(400, 400);
  }
}

void debugSensorDistance()
{
  debugOutput = "d";
  debugOutput = debugOutput + srSensorFront1Distance + " | " + srSensorFront2Distance + " | " + srSensorFront3Distance + " | " + srSensorLeft1Distance + " | " + srSensorLeft2Distance + " | " + lrSensorRight1Distance;
  Serial.println(debugOutput);
}
