// Motor Library
// https://github.com/pololu/dual-vnh5019-motor-shield
#include "DualVNH5019MotorShield.h"

// Sensor Library
// https://github.com/guillaume-rico/SharpIR
#include "ZSharpIR.h"

// Definition of Parameters
#define IR A1
#define sharp_sensor_model 20150

DualVNH5019MotorShield md;
ZSharpIR SharpIR(IR, sharp_sensor_model)

// ===== setup() =====
void setup()
{
  Serial.begin(115200);
  md.init();
}

// ===== loop() =====
void loop()
{
  Serial.println(SharpIR.distance());
}
