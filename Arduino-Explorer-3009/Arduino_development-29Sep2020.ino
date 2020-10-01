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
#define lrSensorRight1 A4   //PS5

// Initialisation
DualVNH5019MotorShield md;
SharpIR SRSensorFront1(SharpIR:: GP2Y0A21YK0F, srSensorFront1);
SharpIR SRSensorFront2(SharpIR:: GP2Y0A21YK0F, srSensorFront2);
SharpIR SRSensorLeft1(SharpIR:: GP2Y0A21YK0F, srSensorLeft1);
SharpIR SRSensorLeft2(SharpIR:: GP2Y0A21YK0F, srSensorLeft2);
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

float kpLeftME =  2.7; //2.5;
float kiLeftME = 0.1835;
float kdLeftME = 0.28;
float kpRightME = 2.10; //latest: 2.30 //2.43; // 2
float kiRightME = 0.13; //latest: 0.13 //0.15;
float kdRightME = 0.20; //latest: 0.20 //0.18; //0.2;
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
double setPoint = 80;

/*delay at start for switch turn on*/
boolean startupFlag = true;
boolean flag = true;

float timeRPM;
float ptimeRPM;

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
  String cc;
  // left(2);
//  printDistance();
//  delay(1000);
  if (Serial.available() > 0)
  {
    cc = char(Serial.read());
    if(cc == "L"){
      left(0.5);
      delay(500);
      printDistance();
    }
    if(cc == "A"){
      left(1);
//      left(0.5);
      delay(500);
      printDistance();
    }
    if(cc == "F"){
      goStraight1Grid();
      delay(1000);
      printDistance();
      restartPID();
    }
    if(cc == "Z"){
    printDistance();
    }
    if(cc == "E"){
    printDistance();
    }
  }
  
  
  timeRPM = millis();
  if (timeRPM - ptimeRPM > 50) {
//    Serial.print("RIGHT PID ");
//    Serial.println(PIDOutputRightME);
//    Serial.print("LEFT PID ");
//    Serial.println(PIDOutputLeftME);
//    Serial.println(Input_MR);
    ptimeRPM = timeRPM;
  }
}

void printDistance(){
  Serial.print("|");
  // Serial.print(SRSensorFront1.getDistance());
  Serial.print(SRSensorFront1Distance(10));
  Serial.print("|");
  Serial.print(SRSensorFront2Distance(10));
  Serial.print("|");
  Serial.print(SRSensorLeft1.getDistance());
  Serial.print("|");
  Serial.print(SRSensorLeft2.getDistance());
  Serial.print("|");
  Serial.print(LRSensorRight1.getDistance());
  Serial.println("|");
}

double SRSensorFront1Distance(int n) {
  double totalDistance = 0;
  int i = 0;
  for (i = 0; i < n; i++) {
    totalDistance += SRSensorFront1.getDistance();
  }
  return (totalDistance / n);
}

double SRSensorFront2Distance(int n) {
  double totalDistance = 0;
  int i = 0;
  for (i = 0; i < n; i++) {
    totalDistance += SRSensorFront2.getDistance();
  }
  return (totalDistance / n);
}

void leftEncoderInc()
{
  counterLeft++;
  if (counterLeft == 1) {
    startTimeLeftME = micros();
  } else if (counterLeft == 5) {
    endTimeLeftME = micros();
    timeTakenLeftME = (endTimeLeftME - startTimeLeftME) / 4;
    RPMLeft = calculateRPM(timeTakenLeftME);
    counterLeft = 0;
  }
}

void rightEncoderInc()
{
  counterRight++;
  if (counterRight == 1) {
    startTimeRightME = micros();
  } else if (counterRight == 5) {
    endTimeRightME = micros();
    timeTakenRightME = (endTimeRightME - startTimeRightME) / 4;
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
  k2LeftME = - kpLeftME - (2 * kdLeftME);
  k3LeftME = kdLeftME;
  k1RightME = kpRightME + kiRightME + kdRightME;
  k2RightME = - kpRightME - (2 * kdRightME);
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

  //Restart PID Value
  if (PIDOutputLeftME >4) {
    PIDOutputLeftME = 0;
    previousPIDOutputLeftME = 0;
    previousPreviousErrorLeftME = 0;
    previousErrorLeftME = 0;
    errorLeftME = 0;
  }
  if (PIDOutputRightME >4) {
    PIDOutputRightME = 0;
    previousPIDOutputRightME = 0;
    previousPreviousErrorRightME = 0;
    previousErrorRightME = 0;
    errorRightME = 0;
  }
}

void goStraight1Grid() {
  Serial.println("In goStraight1Grid");
//  long distance = 10700;
  long distance = 150000;
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
  PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setPoint);
  md.setSpeeds(PIDOutputRightME * 250, PIDOutputLeftME * 250);
//swapped for testing :
//md.setSpeeds(PIDOutputLeftME * 200, PIDOutputRightME * -200);
//  Serial.print("RPM Right ");
//  Serial.println(RPMRight);
//  Serial.print("RPM Left ");
//  Serial.println(RPMLeft);

  Serial.print("RIGHT PID ");
  Serial.println(PIDOutputRightME);
  Serial.print("LEFT PID ");
  Serial.println(PIDOutputLeftME);
  delayMicroseconds(4230);
}

//testing turn left code
void left(float n)
{
  double totalDis = 0;
  int count = 0;
  boolean  flag = true;

  while (flag) {
    //half turn : 30000
    //1/4 turn : 14000
    //3000
    int distanceLimit = 0;
    if(n == 0.25){
      distanceLimit = 16400;
    }
    else if(n > 0.25 ){
      double k = n / 0.25;
      distanceLimit = 16400 + (250 * k);
      // distanceLimit = 17800;
    }
    
    if (totalDis >= distanceLimit)
    {
      md.setBrakes(375, -375);
      totalDis = 0;
      count++;

      if (count == 4 * n) {
        count = 0;
        flag = false;
      }
    }

    else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setPoint);
//      PIDCalculation(12, 0.14, 9, 12, 0.147, 9, 80);
//      Serial.print("RPM Right ");
//      Serial.println(RPMRight);
//      Serial.print("RPM Left ");
//      Serial.println(RPMLeft);
//      Serial.println("PID Left: ");
//      Serial.println(PIDOutputLeftME);
//      Serial.println("PID Right: ");
//      Serial.println(PIDOutputRightME);
//      md.setSpeeds(PIDOutputLeftME * 200, PIDOutputRightME * -200);
      md.setSpeeds(250, -250);
      totalDis = totalDis + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

void right(float n)
{
  double totalDis = 0;
  int count = 0;
  boolean  flag = true;

  while (flag) {
    // half turn : 30000
    // 1/4 turn : 14000
    if (totalDis >= 12000)
    {
      md.setBrakes(-375, 375);
      totalDis = 0;
      count++;

      if (count == 4 * n) {
        count = 0;
        flag = false;
      }
    }

    else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setPoint);
//      PIDCalculation(12, 0.14, 9, 12, 0.147, 9, 80);
      Serial.print("RPM Right ");
      Serial.println(RPMRight);
      Serial.print("RPM Left ");
      Serial.println(RPMLeft);
      Serial.println("PID Left: ");
      Serial.println(PIDOutputLeftME);
      Serial.println("PID Right: ");
      Serial.println(PIDOutputRightME);
//      md.setSpeeds(PIDOutputLeftME * 200, PIDOutputRightME * -200);
      md.setSpeeds(-250, 250);
      totalDis = totalDis + RPMLeft + RPMRight;
      delayMicroseconds(4230);
    }
  }
}

void restartPID(){
  previousPreviousErrorRightME = previousErrorRightME = errorRightME = 0;
  previousPreviousErrorLeftME = previousErrorLeftME = errorLeftME = 0;
  previousPIDOutputLeftME = PIDOutputLeftME = 0;
  previousPIDOutputRightME = PIDOutputRightME = 0;
  RPMLeft = RPMRight = 0;
}
