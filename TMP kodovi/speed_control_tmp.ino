#include <Math.h>

#define MOTOR1PINA 9
#define MOTOR1PINB 10
#define MOTOR2PINA 7
#define MOTOR2PINB 8
#define MOTOR1PWM 5
#define MOTOR2PWM 6

#define MOTOR1INTA 2
#define MOTOR1INTB 11
#define MOTOR2INTA 3
#define MOTOR2INTB 12

#define INTPIN1 2
#define INTPIN2 3

#define MIL 1000

float brzinaD = 0;
float brzinaL = 0;
float predhodnaBrzinaD = 0;
float predhodnaBrzinaL = 0;
float ubrzanje = 10;

long vremeD = 0;
long vremeL = 0;

long predhodniMilisL = 0;
long predhodniMilisD = 0;
long milis = 0;
long predhodniMilis = 0;

int predhodniCountL = 0;
int predhodniCountD = 0;

int out1 = 100;
int out2 = 123;

int rightCount = 0;
int leftCount = 0;

float racunajBrzinu(int vreme, float *predhodnaBrzina){

	float brzina = ubrzanje*vreme/MIL + *predhodnaBrzina;

	return brzina;

}

int racunajBrzinu2(int count, int *predhodniCount, long milis, long predhodniMilis){

  int a;
  a = (count - *predhodniCount)/ (1000/(milis-predhodniMilis));
  *predhodniCount = count;
  return a;

}

void balansiraj_brzine(float brz1, float brz2, int *out1, int *out2){
	if(abs(brz1 - brz2) >= 5){
		if(brz1 > brz2){
			*out2 += 5;
		}else{
			*out1 += 5;
		}
	}else{
    if(brz1 > brz2){
      *out1 -= 5;
    }else{
      *out2 -= 5;
    }
	}
}

void vozi_mishko(){
	//brzinaL = racunajBrzinu(vremeL,&predhodnaBrzinaL);
	//brzinaD = racunajBrzinu(vremeD,&predhodnaBrzinaD);

	//balansiraj_brzine(brzinaL,brzinaD,&out1,&out2);
}

void debug(){

	Serial.print("enkoder1: ");
	Serial.print(rightCount);
	Serial.print(" enkoder2: ");
	Serial.print(leftCount);

	Serial.print(" || brzina1: ");
	Serial.print(brzinaD);
	Serial.print(" brzina2: ");
	Serial.print(brzinaL);

	Serial.print(" || out1: ");
	Serial.print(out1);
	Serial.print(" out2: ");
	Serial.print(out2);

  Serial.print(" || enc1: ");
  Serial.print(digitalRead(MOTOR1INTA));
  Serial.print(" enc2: ");
  Serial.println(digitalRead(MOTOR1INTB));

}

void setup() {

  Serial.begin(9600);

  pinMode(MOTOR1PINA, OUTPUT);
  pinMode(MOTOR1PINB, OUTPUT);
  pinMode(MOTOR2PINA, OUTPUT);
  pinMode(MOTOR2PINB, OUTPUT);
  pinMode(MOTOR1PWM, OUTPUT);
  pinMode(MOTOR2PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(2), rightEncoderEvent, CHANGE);      //interupti za enkodere
  attachInterrupt(digitalPinToInterrupt(3), leftEncoderEvent, CHANGE);

}

void idi_napred(){

  digitalWrite(MOTOR1PINA,1);
  digitalWrite(MOTOR1PINB,0);
  analogWrite(MOTOR1PWM,out1);
  digitalWrite(MOTOR2PINA,1);
  digitalWrite(MOTOR2PINB,0);
  analogWrite(MOTOR2PWM,out2);

}

void idi_nazad(){

  digitalWrite(MOTOR1PINA,0);
  digitalWrite(MOTOR1PINB,1);
  analogWrite(MOTOR1PWM,out1);
  digitalWrite(MOTOR2PINA,0);
  digitalWrite(MOTOR2PINB,1);
  analogWrite(MOTOR2PWM,out2);

}

void stani(){
  digitalWrite(MOTOR1PINA,0);
  digitalWrite(MOTOR1PINB,0);
  analogWrite(MOTOR1PWM,0);
  digitalWrite(MOTOR2PINA,0);
  digitalWrite(MOTOR2PINB,0);
  analogWrite(MOTOR2PWM,0);
}

void skreni_levo(){
  digitalWrite(MOTOR1PINA,0);
  digitalWrite(MOTOR1PINB,0);
  analogWrite(MOTOR1PWM,0);
  digitalWrite(MOTOR2PINA,1);
  digitalWrite(MOTOR2PINB,0);
  analogWrite(MOTOR2PWM,100);
}

void skreni_desno(){
  digitalWrite(MOTOR1PINA,1);
  digitalWrite(MOTOR1PINB,0);
  analogWrite(MOTOR1PWM,100);
  digitalWrite(MOTOR2PINA,0);
  digitalWrite(MOTOR2PINB,0);
  analogWrite(MOTOR2PWM,0);
}

char ch;

void loop() {

	//debug();

  if(millis() - predhodniMilis >= 250){
    milis = millis();
    brzinaD = racunajBrzinu2(rightCount,&predhodniCountD, milis, predhodniMilis);
    brzinaL = racunajBrzinu2(leftCount,&predhodniCountL, milis, predhodniMilis);

    predhodniMilis = millis();
  }
  
  if(Serial.available() > 0){
  	ch = Serial.read();
  }

  if(ch == 'w'){
  	vozi_mishko();
  	idi_napred();
  }else if (ch == 'a'){
  	vozi_mishko();
  	skreni_levo();
  }else if(ch == 's'){
  	vozi_mishko();
  	idi_nazad();
  }else if (ch == 'd'){
  	vozi_mishko();
  	skreni_desno();
  }else if (ch == 'q'){
  	stani();
  }else if (ch == 'z'){
    out1++;
  }else if (ch == 'x'){
    out2++;
  }else if (ch == 'y'){
    out1--;
  }else if (ch == 'u'){
    out2--;
  }

}

void rightEncoderEvent() {
 
  vremeD = micros() - predhodniMilisD;
  predhodniMilisD = micros();

  if (digitalRead(MOTOR1INTA) == HIGH) {
    if (digitalRead(MOTOR1INTB) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  } else {
    if (digitalRead(MOTOR1INTB) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  }
}

void leftEncoderEvent() {
  vremeL = micros() - predhodniMilisL;
  predhodniMilisL = micros();
  
  if (digitalRead(MOTOR2INTA) == HIGH) {
    if (digitalRead(MOTOR2INTB) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(MOTOR2INTB) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
