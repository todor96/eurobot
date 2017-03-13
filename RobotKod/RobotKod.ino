#include "MotorControl.h"

//Brojevi obrtaja tockova
volatile long leftCount = 0;
volatile long rightCount = 0;

//Predjeni put tockova
volatile double leftDistance = 0;
volatile double rightDistance = 0;

//ulaz sa serijskog porta
int setpointInput = 0;

//Inicijalizacija biblioteke za kontrolu motora
MotorControl motorControl(&leftCount, &rightCount, &leftDistance, &rightDistance);


void setup() {
  //Inicijalizacija serijskog porta
  Serial.begin(9600);
  
  //Inicijalizacija prekida za enkodere
  attachInterrupt(digitalPinToInterrupt(20), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), rightEncoderEvent, CHANGE);
}


void loop() {
  //Ucitavanje setpointa sa serijskog porta
  readSetpointSerial();
  
  //Pokretanje robota za odredjeni setpoint
  motorControl.driveRotations(setpointInput);
  //motorControl.driveCm(50);
  
  //Stampanje svih promenljivih na serijski port
  printDebug();
}



//Stampanje svih promenljivih na serijski port
void printDebug() {

  Serial.print("  PidOutputL  ");
  Serial.print(motorControl.myPidOutputL);

  Serial.print("  PidOutputR ");
  Serial.print(motorControl.myPidOutputR);

  Serial.print("  PidSetpointL  ");
  Serial.print(motorControl.myPidSetpointL);

  Serial.print("  PidSetpointR  ");
  Serial.print(motorControl.myPidSetpointR);

  Serial.print("  PidInputL  ");
  Serial.print(motorControl.myPidInputL);

  Serial.print("  PidInputR  ");
  Serial.print(motorControl.myPidInputR);

  Serial.print("  enkoderL  ");
  Serial.print(leftCount);

  Serial.print("  enkoderR ");
  Serial.print(rightCount);

  Serial.print("  dirR  ");
  Serial.print(motorControl.myMotorDirectionR);

  Serial.print("  dirL ");
  Serial.print(motorControl.myMotorDirectionL);

  Serial.print("  leftDistance  ");
  Serial.print(leftDistance);
  
  Serial.print("  rightDistance  ");
  Serial.println(leftDistance);

}

//Ucitavanje setpointa sa serijskog porta
void readSetpointSerial(){
  if (Serial.available() > 0) {
    setpointInput = Serial.parseInt();
    Serial.read();

    Serial.print("setpoint");
    Serial.println(setpointInput);
  }
}

//Funkcija interrupt-a za levi enkoder
void leftEncoderEvent(){
  motorControl.leftEncoderEvent();
}

//Funkcija interrupt-a za desni enkoder
void rightEncoderEvent(){
  motorControl.rightEncoderEvent();
}
