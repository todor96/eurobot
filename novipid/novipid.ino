
#include <PID_v1.h>

#define RH_ENCODER_A 18
#define RH_ENCODER_B 19

#define LH_ENCODER_A 20
#define LH_ENCODER_B 21

// Motor desni
int dirRPinA = 2;
int dirRPinB = 3;
int speedPinR = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor levi
int dirLPinA = 5;
int dirLPinB = 4;
int speedPinL = 10; // Needs to be a PWM pin to be able to control motor speed

//Define Variables we'll be connecting to
double SetpointR, InputR, OutputR;
double SetpointL, InputL, OutputL;

volatile long leftCount = 0;
volatile long rightCount = 0;

int motorDirectionR;
int motorDirectionL;

//Specify the links and initial tuning parameters
PID myPID_L(&InputL, &OutputL, &SetpointL,2,1,1, DIRECT);
PID myPID_R(&InputR, &OutputR, &SetpointR,2,1,1, DIRECT);

int setpointInput=0;

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
    setpointInput = Serial.parseInt();
    Serial.read();
    

              Serial.print("setpoint");
              Serial.print(setpointInput);

  }

    if(setpointInput > leftCount){
      InputL = leftCount;
      SetpointL = setpointInput;
      motorDirectionL = 1;
    }else if (setpointInput < leftCount){
      InputL = setpointInput;
      SetpointL = leftCount;
      motorDirectionL = -1;
    }
    
    if(setpointInput > rightCount){
      InputR = rightCount;
      SetpointR = setpointInput;
      motorDirectionR = 1;
    }else if(setpointInput < rightCount){
      InputR = setpointInput;
      SetpointR = rightCount;
      motorDirectionR = -1;
    }
    
    myPID_L.Compute();
    myPID_R.Compute();

              Serial.print("  OutputL  ");
              Serial.print(OutputL);

              Serial.print("  OutputR ");
              Serial.print(OutputR);



              Serial.print("  enkoderR  ");
              Serial.print(rightCount);

              Serial.print("  enkoderL ");
              Serial.print(leftCount);

              Serial.print("  dirR  ");
              Serial.print(motorDirectionR);

              Serial.print("  dirL ");
              Serial.println(motorDirectionL);

  rideMuthepucker(motorDirectionR,dirRPinA,dirRPinB);
  rideMuthepucker(motorDirectionL,dirLPinA,dirLPinB);

//  if(rightCount > SetpointR){
//    digitalWrite(dirRPinA, LOW);
//    digitalWrite(dirRPinB, HIGH);
//  }else if(rightCount < SetpointR){
//    digitalWrite(dirRPinA, HIGH);
//    digitalWrite(dirRPinB, LOW);
//  }
//
//  if(leftCount > SetpointL){
//    digitalWrite(dirLPinA, HIGH);
//    digitalWrite(dirLPinB, LOW);
//  }else if(leftCount < SetpointL){
//    digitalWrite(dirLPinA, LOW);
//    digitalWrite(dirLPinB, HIGH);
//  }
  if(abs(setpointInput-leftCount)>20){
  analogWrite(speedPinL,OutputL);
  }
  if(abs(setpointInput-rightCount)>20){
  analogWrite(speedPinR,OutputR);
  }
  
}

void swap(int *a, int *b){
  int p;
  p = *a;
  *a = *b;
  *b = p;
}

void rideMuthepucker(int dir, int pinA, int pinB){
    /*if(dir == -1){
      swap(&pinA,&pinB);
    }*/
    if(dir>0){
      digitalWrite(pinA, HIGH);
      digitalWrite(pinB, LOW);
    }
    else if(dir<0){
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, HIGH);
    }
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
