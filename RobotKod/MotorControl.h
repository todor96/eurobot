#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h" 
#include "pins_arduino.h"
#endif

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#define DIR_FW 1        //kretanje napred
#define DIR_BW -1       //kretanje nazad

#define ABS_ERROR 5    //Absolutna greska enkodera

class MotorControl {
  public:
    MotorControl(int, int, int, int, int, int);
    //Motor desni
    int myDirRPinA;
    int myDirRPinB;
    int mySpeedPinR;    //PWM pin za desni motor

    //Motor levi
    int myDirLPinA;
    int myDirLPinB;
    int mySpeedPinL;   //PWM pin za levi motor

    void setMotorDirection(int dir, int pinA, int pinB);

    void drive(long myLeftCount, long myRightCount, int mySetpointInput, int myOutputL, int myOutputR);

    void stopMotorL();
    void stopMotorR();
    void stopMotors();
};

#endif
