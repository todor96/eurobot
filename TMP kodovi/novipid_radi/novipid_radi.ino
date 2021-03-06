
#include <PID_v1.h>

#define RH_ENCODER_A 18
#define RH_ENCODER_B 19

#define LH_ENCODER_A 20
#define LH_ENCODER_B 21

// Motor 1
int dirRPinA = 2;
int dirRPinB = 3;
int speedPinR = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dirLPinA = 4;
int dirLPinB = 5;
int speedPinL = 10; // Needs to be a PWM pin to be able to control motor speed

//Define Variables we'll be connecting to
double SetpointR, InputR, OutputR;
double SetpointL, InputL, OutputL;

volatile long leftCount = 0;
volatile long rightCount = 0;

//Specify the links and initial tuning parameters
PID myPID_L(&InputL, &OutputL, &SetpointL,2,5,1, DIRECT);
PID myPID_R(&InputR, &OutputR, &SetpointR,2,5,1, DIRECT);

int a=0;

void setup() {
// initialize serial communication @ 9600 baud:
  Serial.begin(9600);

  //Define L298N Dual H-Bridge Motor Controller Pins
    InputR = 0;
    SetpointR = 0;
    InputL = 0;
    SetpointL = 0;

  //turn the PID on
  myPID_L.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);
  
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

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(18), rightEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), leftEncoderEvent, CHANGE);

  Serial.begin(9600);

}

void loop() {

  if(Serial.available() > 0){
    a = Serial.parseInt();
    Serial.read();
    }
    SetpointL = a;
    SetpointR = a;

              Serial.print("setpoint");
              Serial.print(a);
  
  InputL = leftCount;
  myPID_L.Compute();
  InputR = rightCount;
  myPID_R.Compute();


              Serial.print("  OutputL  ");
              Serial.print(OutputL);

              Serial.print("  OutputR ");
              Serial.print(OutputR);



              Serial.print("  enkoderR  ");
              Serial.print(rightCount);

              Serial.print("  enkoderL ");
              Serial.println(leftCount);

  if(rightCount > SetpointR){
    digitalWrite(dirRPinA, LOW);
    digitalWrite(dirRPinB, HIGH);
  }else if(rightCount < SetpointR){
    digitalWrite(dirRPinA, HIGH);
    digitalWrite(dirRPinB, LOW);
  }

  if(leftCount > SetpointL){
    digitalWrite(dirLPinA, HIGH);
    digitalWrite(dirLPinB, LOW);
  }else if(leftCount < SetpointL){
    digitalWrite(dirLPinA, LOW);
    digitalWrite(dirLPinB, HIGH);
  }
  
  analogWrite(speedPinL,OutputL);
  analogWrite(speedPinR,OutputR);
  
}

void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }

}

// encoder event for the interrupt call
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
