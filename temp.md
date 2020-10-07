
// ==================== Checklist ====================

void avoidObstacle()
{
  Serial.println("===== avoidObstacle() =====");
  while (1) {
    getSensorsDistanceRM(sensorSampleSize);
    restartPID();
    if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
      Serial.println("===== [1] =====");
      goStraightNGrids(1);
      delay(500);
    } else if (lrSensorRight1Distance > 20) {
      Serial.println("===== [2] =====");
      turnRightOneGrid();
      delay(500);
      rightFacing = true;
    } else if (srSensorLeft1Distance > 20 && srSensorLeft2Distance > 20) {
      Serial.println("===== [3] =====");
      turnLeftOneGrid();
      delay(500);
      leftFacing = true;
    }

    if (rightFacing == true) {

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [4] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [5] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      Serial.println("===== [6] =====");
      turnLeftOneGrid();
      delay(500);

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [7] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [8] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [9] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [10] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      Serial.println("===== [13] =====");
      turnLeftOneGrid();
      delay(500);

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [14] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [15] =====");
        goStraightNGrids(1);
        delay(500);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      Serial.println("===== [16] =====");
      turnRightOneGrid();
      delay(500);

      rightFacing = false;

    }

    if (leftFacing == true) {

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      turnRightOneGrid();
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      turnRightOneGrid();
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      goStraightNGrids(1);
      delay(250);

      restartPID();
      turnLeftOneGrid();
      delay(250);

      leftFacing = false;

    }

  }
}


void halfTurnLeftOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance > turnGridDistance) {
      md.setBrakes(400, 400);
      break;
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(PIDOutputRightME * 150, -PIDOutputLeftME * 150);
      delayMicroseconds(1500);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void halfTurnRightOneGrid()
{
  totalDistance = 0;
  while (1) {
    if (totalDistance > turnGridDistance) {
      md.setBrakes(400, 400);
      break;
    } else {
      PIDCalculation(kpLeftME, kiLeftME, kdLeftME, kpRightME, kiRightME, kdRightME, setpoint);
      md.setSpeeds(-PIDOutputRightME * 150, PIDOutputLeftME * 150);
      delayMicroseconds(1500);
      totalDistance = totalDistance + RPMLeft + RPMRight;
    }
  }
}

void tiltAvoidObstacle()
{
  Serial.println("===== tiltAvoidObstacle() =====");
  while (1) {
    getSensorsDistanceRM(sensorSampleSize);
    restartPID();
    if (srSensorFront1Distance > 18 && srSensorFront2Distance > 18 && srSensorFront3Distance > 18) {
      Serial.println("===== [1] =====");
      goStraightNGrids(1);
      delay(125);
    } else if (lrSensorRight1Distance > 20) {
      Serial.println("===== [2] =====");
      halfTurnRightOneGrid();
      delay(125);
      rightFacing = true;
    } else if (srSensorLeft1Distance > 20 && srSensorLeft2Distance > 20) {
      Serial.println("===== [3] =====");
      halfTurnLeftOneGrid();
      delay(125);
      leftFacing = true;
    }

    if (rightFacing == true) {

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [4] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [5] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [6] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [7] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      Serial.println("===== [8] =====");
      halfTurnLeftOneGrid();
      delay(125);

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      Serial.println("===== [9] =====");
      halfTurnLeftOneGrid();
      delay(250);

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [10] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [11] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [12] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      if (srSensorFront1Distance > 11 && srSensorFront2Distance > 11 && srSensorFront3Distance > 11) {
        Serial.println("===== [12] =====");
        goStraightNGrids(1);
        delay(125);
      }

      getSensorsDistanceRM(sensorSampleSize);
      restartPID();
      halfTurnRightOneGrid();
      delay(125);

      rightFacing = false;

    }

  }
}
