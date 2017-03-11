#include <PID_v1.h>
#include "MotorControl.h"
//Enkoderi za desni tocak
#define RH_ENCODER_A 18
#define RH_ENCODER_B 19

//Enkoderi za levi tocak
#define LH_ENCODER_A 20
#define LH_ENCODER_B 21


//Min i max PID output-a
#define PID_MAX 50
#define PID_MIN 0


//Motor desni
int dirRPinA = 2;
int dirRPinB = 3;
int speedPinR = 9;    //PWM pin za desni motor

//Motor levi
int dirLPinA = 7;
int dirLPinB = 6;
int speedPinL = 11;   //PWM pin za levi motor

//Promenljive za PID
double SetpointR, InputR, OutputR;
double SetpointL, InputL, OutputL;

//Parametri PID-a
double Kp = 2;
double Ki = 1;
double Kd = 1;

//brojevi obrtaja tockova
volatile long leftCount = 0;
volatile long rightCount = 0;
volatile double leftDistance = 0;
volatile double rightDistance = 0;

//pravci motora
int motorDirectionR;
int motorDirectionL;

//Specify the links and initial tuning parameters
PID PID_L(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID PID_R(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

//Inicijalizacija biblioteke za kontrolu motora
MotorControl motorControl(dirLPinA, dirLPinB, dirRPinA, dirLPinB, speedPinL, speedPinR);


//ulaz sa serijskog porta
int setpointInput = 0;

void setup() {

  Serial.begin(9600);

  //Inicijalizacija PID inputa i setpointa
  InputR = 0;
  SetpointR = 0;
  InputL = 0;
  SetpointL = 0;

  //Ukljucivanje PID-ova
  PID_L.SetMode(AUTOMATIC);
  PID_R.SetMode(AUTOMATIC);

  //Postavljanje min i max outputa PID-a
  PID_L.SetOutputLimits(PID_MIN, PID_MAX);
  PID_R.SetOutputLimits(PID_MIN, PID_MAX);

  //Inicijalizacija pinova
  pinMode(dirRPinA, OUTPUT);
  pinMode(dirRPinB, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(dirLPinA, OUTPUT);
  pinMode(dirLPinB, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

  //Inicijalizacija prekida za enkodere
  attachInterrupt(digitalPinToInterrupt(18), rightEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), leftEncoderEvent, CHANGE);

  Serial.begin(9600);

}

void loop() {
  //Ucitavanje setpointa sa serijskog porta
  if (Serial.available() > 0) {
    setpointInput = Serial.parseInt();
    Serial.read();

    Serial.print("setpoint");
    Serial.println(setpointInput);


  }

  //Racunanje inputa za PID
  calculatePidInput();

  //racunanje PID-a
  PID_L.Compute();
  PID_R.Compute();

  motorControl.setMotorDirection(motorDirectionR, dirRPinA, dirRPinB);
  motorControl.setMotorDirection(motorDirectionL, dirLPinA, dirLPinB);

  motorControl.drive(leftCount, rightCount, setpointInput, OutputL, OutputR, &leftDistance, &rightDistance);
  //TODO: srediti racunanje PID za driveCm
  // motorControl.driveCm(300, leftCount, rightCount, OutputL, OutputR, &leftDistance, &rightDistance);

  //stampanje svih promenljivih na serijski port
  printDebug();
}

//racunanje obrtaja desnog enkodera
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  }
}

//racunanje obrtaja levog enkodera
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }

}

//TODO: ubaciti calculate PID u MotorConrol.cpp, tako da moze da racuna i za drive i za drive Cm
void calculatePidInput() {
  if (setpointInput > leftCount) {                            //u slucaju da je setpoint veci od trenutne pozicije
    InputL = leftCount;
    SetpointL = setpointInput;
    motorDirectionL = 1;
  }
  else if (setpointInput < leftCount) {                       //u slucaju da je setpoint manji od trenutne pozicije, kada robot treba da ide unazad
    InputL = setpointInput - (leftCount - setpointInput);     //input se postavi da je manji od setpointa za razliku trenutne pozicije i setpointa
    SetpointL = setpointInput;                                //a smer motora se okrene
    motorDirectionL = -1;
  }

  if (setpointInput > rightCount) {
    InputR = rightCount;
    SetpointR = setpointInput;
    motorDirectionR = DIR_FW;
  } else if (setpointInput < rightCount) {
    InputR = setpointInput - (rightCount - setpointInput);;
    SetpointR = setpointInput;
    motorDirectionR = DIR_BW;
  }

}

void printDebug() {

  Serial.print("  OutputL  ");
  Serial.print(OutputL);

  Serial.print("  OutputR ");
  Serial.print(OutputR);

  Serial.print("  SetpointL  ");
  Serial.print(SetpointL);

  Serial.print("  SetpointR  ");
  Serial.print(SetpointR);

  Serial.print("  InputL  ");
  Serial.print(InputL);

  Serial.print("  InputR  ");
  Serial.print(InputR);

  Serial.print("  enkoderR  ");
  Serial.print(rightCount);

  Serial.print("  enkoderL ");
  Serial.print(leftCount);

  Serial.print("  dirR  ");
  Serial.print(motorDirectionR);

  Serial.print("  dirL ");
  Serial.println(motorDirectionL);

  Serial.print("  leftDistance  ");
  Serial.println(leftDistance);

}

