#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>

#define IRPin A0


SharpIR sensor1(SharpIR:: GP2Y0A21YK0F, IRPin);
DualVNH5019MotorShield md;

int distance;

void setup()
{
  Serial.begin(9600);
  int distance = 0;
  int val = 0;
  
  md.init();
}
 
void loop()
{
  //testing IR sensors
  //val = analogRead(IRPin);
  distance = sensor1.getDistance();
  
  Serial.print("distance: ");
  Serial.println(distance);
  delay(1000);


}
