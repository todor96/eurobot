#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "MotorControl.h"


//MOTORI **********************************************************************

//Levi motor
int myDirLPinA = 7;                       //Pin A za levi motor
int myDirLPinB = 6;                       //Pin B za levi motor
int mySpeedPinL = 11;                     //PWM pin za levi motor
int myMotorDirectionL;                    //Pravac levog motora
volatile long *myLeftCount;               //Broj obrtaja enkodera levog motora
volatile double* myLeftDistance;          //Predjeni put levog tocka

//Desni motor
int myDirRPinA = 2;                       //Pin A za desni motor
int myDirRPinB = 3;                       //Pin B za desni motor
int mySpeedPinR = 9;                      //PWM pin za desni motor
int myMotorDirectionR;                    //Pravac Desnog motora
volatile long *myRightCount;              //Broj obrtaja desnog enkodera
volatile double* myRightDistance;         //Predjeni put desnog tocka

boolean standing = true;


//PID ************************************************************************

//Inputi za PID
double myPidInputL = 0;                   //Input za PID levog motora
double myPidInputR = 0;                   //Input za PID desnog motora

//Outputi za PID
double myPidOutputL = 0;                  //Output za PID levog motora
double myPidOutputR = 0;                  //Output za PID desnog motora

//Setpointovi za PID
double myPidSetpointL = 0;                //Setpoint za PID levog motora
double myPidSetpointR = 0;                //Setpoint za PID desnog motora

//Inicijalizacija PID_v1 biblioteke

PID *PID_L;                               //PID za levi tocak
PID *PID_R;                               //PID za desni tocak




MotorControl :: MotorControl(volatile long* leftCount, volatile long* rightCount, volatile double* leftDistance, volatile double* rightDistance) {

  //Levi motor
  myDirLPinA = 7;                       //Pin A za levi motor
  myDirLPinB = 6;                       //Pin B za levi motor
  mySpeedPinL = 11;                     //PWM pin za levi motor

  //Desni motor
  myDirRPinA = 2;                       //Pin A za desni motor
  myDirRPinB = 3;                       //Pin B za desni motor
  mySpeedPinR = 9;


  //Inputi za PID
  myPidInputL = 0;                   //Input za PID levog motora
  myPidInputR = 0;                   //Input za PID desnog motora

  //Outputi za PID
  myPidOutputL = 0;                  //Output za PID levog motora
  myPidOutputR = 0;                  //Output za PID desnog motora

  //Setpointovi za PID
  myPidSetpointL = 0;                //Setpoint za PID levog motora
  myPidSetpointR = 0;                //Setpoint za PID desnog motora

  myLeftCount = leftCount;
  myRightCount = rightCount;
  myLeftDistance = leftDistance;
  myRightDistance = rightDistance;

  //Inicijalizacija PID-a
  PID_L = new PID(&myPidInputL, &myPidOutputL, &myPidSetpointL, KP, KI, KD, DIRECT);
  PID_R = new PID(&myPidInputR, &myPidOutputR, &myPidSetpointR, KP, KI, KD, DIRECT);

  //Ukljucivanje PID-ova
  PID_L->SetMode(AUTOMATIC);
  PID_R->SetMode(AUTOMATIC);

  //Postavljanje min i max outputa PID-a
  PID_L->SetOutputLimits(PID_MIN, PID_MAX);
  PID_R->SetOutputLimits(PID_MIN, PID_MAX);

  //Inicijalizacija pinova
  pinMode(myDirRPinA,  OUTPUT);
  pinMode(myDirRPinB,  OUTPUT);
  pinMode(mySpeedPinR, OUTPUT);
  pinMode(myDirLPinA,  OUTPUT);
  pinMode(myDirLPinB,  OUTPUT);
  pinMode(mySpeedPinL, OUTPUT);

  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

}



//KONTROLA MOTORA ******************************************************************************************************


void MotorControl :: setMotorDirectionL(int dir) {                  //Postavljanje smera obrtanja motora levog motora
  myMotorDirectionL = dir;
  if (dir == DIR_FW) {
    digitalWrite(myDirLPinA, HIGH);
    digitalWrite(myDirLPinB, LOW);
  }
  else if (dir == DIR_BW) {
    digitalWrite(myDirLPinA, LOW);
    digitalWrite(myDirLPinB, HIGH);
  }
}


void MotorControl :: setMotorDirectionR(int dir) {                  //Postavljanje smera obrtanja motora levog motora
  myMotorDirectionR = dir;
  if (dir == DIR_FW) {
    digitalWrite(myDirRPinA, HIGH);
    digitalWrite(myDirRPinB, LOW);
  }
  else if (dir == DIR_BW) {
    digitalWrite(myDirRPinA, LOW);
    digitalWrite(myDirRPinB, HIGH);
  }
}


void MotorControl :: setMotorDirections(int dirL, int dirR) {       //Postavljanje smera obrtanja oba motora
  MotorControl :: setMotorDirectionL(dirL);
  MotorControl :: setMotorDirectionR(dirR);
}


void MotorControl :: stopMotorL() {                                 //Zaustavljanje levog motora
  digitalWrite(myDirLPinA, HIGH);
  digitalWrite(myDirLPinB, HIGH);
  analogWrite(mySpeedPinL, 0);
}


void MotorControl :: stopMotorR() {                                 //Zaustavljanje desnog motora
  digitalWrite(myDirRPinA, HIGH);
  digitalWrite(myDirRPinB, HIGH);
  analogWrite(mySpeedPinR, 0);
}


void MotorControl :: stopMotors() {                                 //Zaustavljanje oba motora
  stopMotorL();
  stopMotorR();
}


void MotorControl :: driveRotations(int noOfRotations) {            //Funkcija za kretanje za zadati broj obrtaja enkodera

  standing = false;
  calculatePidInput(noOfRotations);

  PID_L->Compute();                                                 //* Izracunavanje Output-a PID levog motora
  PID_R->Compute();                                                 //  Izracunavanje Output-a PID desnog motora


  if (abs(noOfRotations - *myLeftCount) > ABS_ERROR) {              //* Ako je udaljenost od setpointa veca od dozvoljene
    standing = false;
    analogWrite(mySpeedPinL, myPidOutputL);                         //  greske, pomeraj motore,u suprotnom zaustavi
  }
  else{
    analogWrite(mySpeedPinL, 0);
    standing = true;
  }

  if (abs(noOfRotations - *myRightCount) > ABS_ERROR) {
    standing = false;
    analogWrite(mySpeedPinR, myPidOutputR);
  }
  else{
    analogWrite(mySpeedPinR, 0);
    standing = true;
  }
  calculateDistance();
}


void MotorControl :: driveCm(double len) {                          //Funkcija za kretanje za zadati put u centimetrima
  int noOfRotations = (len * ENCODER_STEPS) / OBIM;

  calculatePidInput(noOfRotations);
  PID_L->Compute();
  PID_R->Compute();

  driveRotations(noOfRotations);
}


void MotorControl :: calculateDistance() {                          //Racunanje predjenog puta tockova
  *myLeftDistance = (1.0 * (*myLeftCount) / ENCODER_STEPS) * OBIM;
  *myRightDistance = (1.0 * (*myRightCount) / ENCODER_STEPS) * OBIM;
}

void MotorControl :: rotateLeft(int input){
    int tmp1 = *myRightCount;
    int tmp2 = *myLeftCount;
    MotorControl :: setMotorDirectionL(DIR_BW);
    MotorControl :: setMotorDirectionR(DIR_FW);
    analogWrite(mySpeedPinL, 255);
    analogWrite(mySpeedPinR, 255);
    while((*myRightCount - tmp1) <= input);
    analogWrite(mySpeedPinL, 0);
    analogWrite(mySpeedPinR, 0);
    *myRightCount = tmp1;
    *myLeftCount = tmp2;
}

//KONTROLA PID-a **********************************************************************************************************************


void MotorControl :: calculatePidInput(int setpoint) {              //Racunanje inputa PID-a i postavljanje smera motora
  if (setpoint > *myLeftCount) {
    myPidInputL = *myLeftCount;
    myPidSetpointL = setpoint;
    setMotorDirectionL(DIR_FW);
  }
  else if (setpoint < *myLeftCount) {                               //* u slucaju da je setpoint manji od trenutne pozicije,
    myPidInputL = setpoint - abs((*myLeftCount - setpoint));        //  kada robot treba da ide unazad, input se postavi da je
    myPidSetpointL = setpoint;                                      //  manji od setpointa za razliku trenutne pozicije i setpointa,
    setMotorDirectionL(DIR_BW);                                     //  a smer motora se okrene
  }

  if (setpoint > *myRightCount) {
    myPidInputR = *myRightCount;
    myPidSetpointR = setpoint;
    setMotorDirectionR(DIR_FW);
  }
  else if (setpoint < *myRightCount) {
    myPidInputR = setpoint - abs((*myRightCount - setpoint));
    myPidSetpointR = setpoint;
    setMotorDirectionR(DIR_BW);
  }
}



//**** KONTROLA ENKODERA ****

void MotorControl :: rightEncoderEvent() {                          //Racunanje obrtaja desnog enkodera
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      (*myRightCount)--;
    } else {
      (*myRightCount)++;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      (*myRightCount)++;
    } else {
      (*myRightCount)--;
    }
  }
}


void MotorControl :: leftEncoderEvent() {                           //Racunanje obrtaja levog enkodera
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      (*myLeftCount)++;
    } else {
      (*myLeftCount)--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      (*myLeftCount)--;
    } else {
      (*myLeftCount)++;
    }
  }
}
