#define RH_ENCODER_A 18
#define RH_ENCODER_B 19

#define LH_ENCODER_A 20
#define LH_ENCODER_B 21

// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 5;
int dir2PinB = 4;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2;
double Setpoint1, Input1, Output1;

volatile long leftCount = 0;
volatile long rightCount = 0;

int a=0;
unsigned long vremeD = 0;
unsigned long vremeL = 0;
unsigned long predhodniMilisD = 0;
unsigned long predhodniMilisL = 0;

int proba=50;

int racunajPid(int input, int setpoint, int output){
  if(input < setpoint){
    output++;
    if(output > 255){
      output = 255;
    }else if(output < 0){
      output = 0;
    }
    return output;
  }else if(input > setpoint){
    output--;
    if(output > 255){
      output = 255;
    }else if(output < 0){
      output = 0;
    }
    return output;
  }
  return output;
}

void setup() {
// initialize serial communication @ 9600 baud:
  Serial.begin(9600);

  //Define L298N Dual H-Bridge Motor Controller Pins
    Input2 = 0;
    Setpoint2 = 0;
    Input1 = 0;
    Setpoint1 = 0;
  
  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);
  pinMode(speedPinB, OUTPUT);

  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(18), rightEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), leftEncoderEvent, CHANGE);

  Serial.begin(9600);

  digitalWrite(dir1PinA,1);
  digitalWrite(dir2PinA,0);
  analogWrite(speedPinA,255);
  delay(1000);
  digitalWrite(dir1PinA,0);
  digitalWrite(dir2PinA,0);
  analogWrite(speedPinA,0);
  
  Serial.print("motor desni, dir1pin a , napred ->");
  Serial.println(rightCount);

  digitalWrite(dir1PinB,1);
  digitalWrite(dir2PinB,0);
  analogWrite(speedPinB,255);
  delay(1000);
  digitalWrite(dir1PinB,0);
  digitalWrite(dir2PinB,0);
  analogWrite(speedPinB,0);
  
  Serial.print("motor levi, dir1pin b , napred ->");
  Serial.println(leftCount);
  delay(5000);
  leftCount = 0;
  rightCount = 0;
}

int ch =1;
int i =0;

void loop() {

  if(Serial.available() > 0){
    a = Serial.parseInt();
    ch=Serial.parseInt();
    Serial.read();
    

             // Serial.print("setpoint");
              //Serial.println(a);

  }
  
    Setpoint1 = a;
    Setpoint2 = a;
  
  Input1 = 1000000/vremeD;
  Output1 = racunajPid(Input1,Setpoint1,Output1);
  Input2 = 1000000/vremeL;
  Output2 = racunajPid(Input2,Setpoint2,Output2);
  //myPID2.Compute();
  
  if(ch == 1){
    digitalWrite(dir1PinA,0);
    digitalWrite(dir1PinB,0);
  }else if (ch == 2){
    digitalWrite(dir1PinA,1);
    digitalWrite(dir1PinB,1);
  }
  digitalWrite(dir2PinA,0);
  digitalWrite(dir2PinB,0);
  analogWrite(speedPinA,Output1);
  analogWrite(speedPinB,Output2);
  //analogWrite(speedPinA,100);
 /* Serial.print("output 1: ");
  Serial.print(Output1);
  Serial.print(" output 2: ");
  Serial.print(Output2);
  Serial.print(" input 1: ");
  Serial.print(Input1);
  Serial.print(" Input2: ");
  Serial.println(Input2);*/

  
  
  //analogWrite(speedPinB,Output1);
  //analogWrite(speedPinA,Output2);
  
}

void rightEncoderEvent() {
 
  //vremeD = ddd - vremeD;
  vremeD = micros() - predhodniMilisD;
  predhodniMilisD = micros();
  if (digitalRead(RH_ENCODER_A) == digitalRead(RH_ENCODER_B)) {
      rightCount--;
    } else {
      rightCount++;
    }
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  vremeL = micros() - predhodniMilisL;
  predhodniMilisL = micros();
  if (digitalRead(LH_ENCODER_A) == digitalRead(LH_ENCODER_B)) {
      leftCount--;
    } else {
      leftCount++;
    } 
}
