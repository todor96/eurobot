#include "MotorControl.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif
#include <math.h>

int myDirRPinA;
int myDirRPinB;
int mySpeedPinR;   //PWM pin za desni motor

//Motor levi
int myDirLPinA;
int myDirLPinB;
int mySpeedPinL;

MotorControl :: MotorControl(int dirLPinA, int dirLPinB, int dirRPinA, int dirRPinB, int speedPinL, int speedPinR) {
  //Motor desni
  myDirRPinA = dirRPinA;
  myDirRPinB = dirRPinB;
  mySpeedPinR = speedPinR;    //PWM pin za desni motor

  //Motor levi
  myDirLPinA = dirLPinA;
  myDirLPinB = dirLPinB;
  mySpeedPinL = speedPinL;   //PWM pin za levi motor
}

void MotorControl :: setMotorDirection(int dir, int pinA, int pinB) {
  if (dir == DIR_FW) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  }
  else if (dir == DIR_BW) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }
}

void MotorControl :: stopMotorL() {
  digitalWrite(myDirLPinA, HIGH);
  digitalWrite(myDirLPinB, HIGH);
  analogWrite(mySpeedPinL, 0);
}

void MotorControl :: stopMotorR() {
  digitalWrite(myDirRPinA, HIGH);
  digitalWrite(myDirRPinB, HIGH);
  analogWrite(mySpeedPinR, 0);
}

void MotorControl :: stopMotors() {
  stopMotorL();
  stopMotorR();
}


void MotorControl :: drive(long myLeftCount, long myRightCount, int mySetpointInput, int myOutputL, int myOutputR, volatile double* myLeftDistance, volatile double* myRightDistance) {
  //ako je udaljenost od setpointa veca od dozvoljene greske, pomeraj motore
  //u suprotnom zaustavi
  if (abs(mySetpointInput - myLeftCount) > ABS_ERROR) {
    analogWrite(mySpeedPinL, myOutputL);
    Serial.println(mySpeedPinL);
  }
  else analogWrite(mySpeedPinL, 0);

  if (abs(mySetpointInput - myRightCount) > ABS_ERROR) {
    analogWrite(mySpeedPinR, myOutputR);
  }
  else analogWrite(mySpeedPinR, 0);
  calculateDistance(myLeftDistance, myRightDistance, myLeftCount, myRightCount);
}

void MotorControl :: driveCm(double len, long myLeftCount, long myRightCount, int myOutputL, int myOutputR, volatile double* myLeftDistance, volatile double* myRightDistance) {
  int setpoint = (len * ENCODER_STEPS) / OBIM;
  MotorControl :: drive(myLeftCount, myRightCount,  setpoint, myOutputL, myOutputR, myLeftDistance, myRightDistance);
}

void MotorControl :: calculateDistance(volatile double* myLeftDistance, volatile double* myRightDistance, long myLeftCount, long myRightCount) {
  *myLeftDistance = (1.0 * myLeftCount / ENCODER_STEPS) * OBIM;
  *myRightDistance = (1.0 * myRightCount / ENCODER_STEPS) * OBIM;
}



