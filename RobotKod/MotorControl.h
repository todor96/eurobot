#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

//Biblioteka za PID
#include <PID_v1.h>



//DEFINOVI ##################################################################



//SMEROVI KRETANJA **********************************************************

#define DIR_FW 1                      //Kretanje unapred
#define DIR_BW -1                     //Kretanje unazad


//PARAMETRI TOCKOVA *********************************************************

#define ABS_ERROR 5                   //Absolutna greska enkodera
#define OBIM 39.8                     //Obim tockova
#define ENCODER_STEPS 150             //Broj koraka enkodera u punom krugu


//PARAMETRI PID-a ***********************************************************

#define PID_MAX 100                    //Maksimalni output PID algoritma
#define PID_MIN 0                     //Minimalni output PID algoritma
#define KP 2.0                        //KP konstanta PID algoritma
#define KI 1.0                        //KI konstanta PID algoritma
#define KD 0.5                        //KD konstanta PID algoritma


//PINOVI ENKODERA ***********************************************************

#define RH_ENCODER_A 18               //Pin A enkodera za desni tocak
#define RH_ENCODER_B 19               //Pin B enkodera za desni tocak
#define LH_ENCODER_A 20               //Pin A enkodera za levi tocak
#define LH_ENCODER_B 21               //Pin B enkodera za desni tocak


class MotorControl {
  public:

    MotorControl(volatile long*, volatile long*, volatile double*, volatile double*);

    //PROMENLJIVE ###########################################################


    //MOTORI ****************************************************************

    //Levi motor
    int myDirLPinA;                   //Pin A za levi motor
    int myDirLPinB;                   //Pin B za levi motor
    int mySpeedPinL;                  //PWM pin za levi motor
    int myMotorDirectionL;            //Pravac levog motora
    volatile long *myLeftCount;       //Broj obrtaja enkodera levog motora
    volatile double* myLeftDistance;  //Predjeni put levog tocka

    //Desni motor
    int myDirRPinA;                   //Pin A za desni motor
    int myDirRPinB;                   //Pin B za desni motor
    int mySpeedPinR;                  //PWM pin za desni motor
    int myMotorDirectionR;            //Pravac Desnog motora
    volatile long *myRightCount;      //Broj obrtaja desnog enkodera
    volatile double* myRightDistance; //Predjeni put desnog tocka


    //PID ******************************************************************

    //Inputi za PID
    double myPidInputL;               //Input za PID levog motora
    double myPidInputR;               //Input za PID desnog motora

    //Outputi za PID
    double myPidOutputL;              //Output za PID levog motora
    double myPidOutputR;              //Output za PID desnog motora

    //Setpointovi za PID
    double myPidSetpointL;            //Setpoint za PID levog motora
    double myPidSetpointR;            //Setpoint za PID desnog motora

    //Inicijalizacija PID_v1 biblioteke

    PID *PID_L;                               //PID za levi tocak
    PID *PID_R;                               //PID za desni tocak



    //FUNKCIJE #################################################################################################################


    //KONTROLA MOTORA **********************************************************************************************************

    void MotorControl :: setMotorDirectionL(int dir);                 //Postavljanje smera obrtanja motora levog motora

    void MotorControl :: setMotorDirectionR(int dir);                 //Postavljanje smera obrtanja motora desnog motora

    void MotorControl :: setMotorDirections(int dirL, int dirR);      //Postavljanje smera obrtanja oba motora

    void MotorControl :: stopMotorL() ;                               //Zaustavljanje levog motora

    void MotorControl :: stopMotorR();                                //Zaustavljanje desnog motora

    void MotorControl :: stopMotors();                                //Zaustavljanje oba motora

    void MotorControl :: driveRotations(int noOfRotations);           //Funkcija za kretanje za zadati broj obrtaja enkodera

    void MotorControl :: driveCm(double len);                         //Funkcija za kretanje za zadati put u centimetrima

    void MotorControl :: calculateDistance();                         //Racunanje predjenog puta tockova


    //KONTROLA PID-a ***********************************************************************************************************

    void MotorControl :: calculatePidInput(int setpoint);             //Racunanje inputa PID-a i postavljanje smera motora


    //KONTROLA ENKODERA ********************************************************************************************************

    void MotorControl :: rightEncoderEvent();                         //Racunanje obrtaja desnog enkodera

    void MotorControl :: leftEncoderEvent();                          //Racunanje obrtaja levog enkodera

};

#endif
