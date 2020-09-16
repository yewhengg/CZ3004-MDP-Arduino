// References
// https://github.com/espressodepresso/CZ3004-MDP-Group-30-2020
// https://github.com/nigelawj/cz3004
// https://github.com/laksh22/MDP/tree/master/Arduino
// https://github.com/SoulXHades/CZ3004-MDP-Arduino
// https://github.com/cheyanniee/Multidisciplinary_Design_Project_CZ3004/tree/master/Arduino
// https://github.com/shiheng7/CZ3004-Arduino/blob/master/MDP_GRP9_Arduino/MDP_GRP9_Arduino.ino

// Robot Layout
//      <srSensorFront1>   <srSensorFront2>
//  <srSensorLeft1>            <srSensorRight1>
//  <md1>                       <md2>
//  <srSensorLeft2>            <lrSensorRight2>

// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"
// M1 = Right Motor, M2 = Left Motor

// Sensor Library
// https://github.com/qub1750ul/Arduino_SharpIR
#include <SharpIR.h>

// Pin Interrupt Library
// https://github.com/NicoHood/PinChangeInterrupt
// #include <PinChangeInterrupt.h>
#include "PinChangeInt.h"

// Parameters Definition
#define motorEncoderRight1 3
#define motorEncoderLeft1 11
#define srSensorFront1 A0   //PS1
#define srSensorFront2 A1   //PS2
#define srSensorLeft1 A2    //PS3
#define srSensorLeft2 A3    //PS4
#define srSensorRight1 A4   //PS5
#define lrSensorRight1 A5   //PS6

// Initialisation
DualVNH5019MotorShield md;
SharpIR SRSensorFront1(SharpIR:: GP2Y0A21YK0F, srSensorFront1);
SharpIR SRSensorFront2(SharpIR:: GP2Y0A21YK0F, srSensorFront2);
SharpIR SRSensorLeft1(SharpIR:: GP2Y0A21YK0F, srSensorLeft1);
SharpIR SRSensorLeft2(SharpIR:: GP2Y0A21YK0F, srSensorLeft2);
SharpIR SRSensorRight1(SharpIR:: GP2Y0A21YK0F, srSensorRight1);
SharpIR LRSensorRight1(SharpIR:: GP2Y0A02YK0F, lrSensorRight1);

// Parameters Declaration
unsigned long startTimeRightME;
unsigned long endTimeRightME;
unsigned long timeTakenRightME;
unsigned long startTimeLeftME;
unsigned long endTimeLeftME;
unsigned long timeTakenLeftME;
double RPMRight;
double RPMLeft;
int counterRight = 0;
int counterLeft = 0;

float kpLeftME = 0.1406;
float kiLeftME = 0.0156;
float kdLeftME = 0.3163;
float kpRightME = 0.1514;
float kiRightME = 0.0168;
float kdRightME = 0.3407;
double k1LeftME = 0;
double k2LeftME = 0;
double k3LeftME = 0;
double k1RightME = 0;
double k2RightME = 0;
double k3RightME = 0;
double errorLeftME = 0;
double errorRightME = 0;
double PIDOutputLeftME = 0;
double previousPIDOutputLeftME = 0;
double PIDOutputRightME = 0;
double previousPIDOutputRightME = 0;
double previousPreviousErrorLeftME = 0;
double previousErrorLeftME = 0;
double previousPreviousErrorRightME = 0;
double previousErrorRightME = 0;
double setPoint = 90;

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

// ==================== loop() ====================
void loop()
{
  // md.setM1Speed(200);
  // md.setM2Speed(400);
  //md.setSpeeds(0, 0);
  md.setSpeeds(400, 400);
  // goStraight1Grid();
  delay(5000);
}

void leftEncoderInc()
{
  counterLeft++;
  if (counterLeft == 1) {
    startTimeLeftME = micros();
  } else if (counterLeft == 101) {
    endTimeLeftME = micros();
    timeTakenLeftME = (endTimeLeftME - startTimeLeftME) / 100;
    RPMLeft = calculateRPM(timeTakenLeftME);
    counterLeft = 0;
  }
}

void rightEncoderInc()
{
  counterRight++;
  if (counterRight == 1) {
    startTimeRightME = micros();
  } else if (counterRight == 101) {
    endTimeRightME = micros();
    timeTakenRightME = (endTimeRightME - startTimeRightME) / 100;
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

void PIDCalculation(float kpLeftME, float kiLeftME, float kdLeftME, float kpRightME, float kiRightME, float kdRightME, double setpoint)
{
  // Calculate Digital PID Parameters
  k1LeftME = kpLeftME + kiLeftME + kdLeftME;
  k2LeftME = kpLeftME - (2 * kdLeftME);
  k3LeftME = kdLeftME;
  k1RightME = kpRightME + kiRightME + kdRightME;
  k2RightME = kpRightME - (2 * kdRightME);
  k3RightME = kdRightME;

  // Calculate Error
  errorLeftME = (setpoint - RPMLeft) / 130;
  errorRightME = (setpoint - RPMRight) / 130;

  // Calculate Digital Control Law
  PIDOutputLeftME = previousPIDOutputLeftME + k1LeftME * errorLeftME + k2LeftME * previousErrorLeftME + k3LeftME * previousPreviousErrorLeftME;
  PIDOutputRightME = previousPIDOutputRightME + k1RightME * errorRightME + k2RightME * previousErrorRightME + k3RightME * previousPreviousErrorRightME;

  // Save Error Values
  previousPIDOutputLeftME = PIDOutputLeftME;
  previousPIDOutputRightME = PIDOutputRightME;
  previousPreviousErrorLeftME = previousErrorLeftME;
  previousErrorLeftME = errorLeftME;
  previousPreviousErrorRightME = previousErrorRightME;
  previousErrorRightME = errorRightME;
}

void goStraight1Grid() {
  Serial.println("In goStraight1Grid");
  long distance = 10980;
  double totalDistance = 0;
  while(1) {
    if (totalDistance >= distance) {
      totalDistance = 0;
      md.setBrakes(400, 400);
      break;
    } else {
      moveForward();
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }  
}

void moveForward() {
  Serial.println("In moveForward()");
  PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setPoint);
  md.setSpeeds(PIDOutputRightME * 250, PIDOutputLeftME * 250);
  delay(50);
}
