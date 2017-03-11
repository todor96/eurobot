#include "MotorControl.h"
#include <Arduino.h>

MotorControl :: MotorControl(int dirLPinA, int dirLPinB, int dirRPinA, int dirRPinB, int speedPinL, int speedPinR) {

  //Motor desni
  int myDirRPinA = dirRPinA;
  int myDirRPinB = dirRPinB;
  int mySpeedPinR = speedPinR;    //PWM pin za desni motor

  //Motor levi
  int myDirLPinA = dirLPinA;
  int myDirLPinB = dirLPinB;
  int mySpeedPinL = speedPinL;   //PWM pin za levi motor

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

