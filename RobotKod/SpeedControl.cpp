#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "SpeedControl.h"
#include <Math.h>

long milis = 0;
long predhodniMilis = 0;

long predhodniCountD = 0;
long predhodniCountL = 0;

float brzinaD = 0;
float brzinaL = 0;

int greskaBrzine = 5;
int balans = 5;

long speedLeftCount = 0;
long speedRightCount = 0;

boolean instanca = false;
SpeedControl *instance;

SpeedControl :: SpeedControl(long *sLeftCount, long *sRightCount){

	speedLeftCount = *sLeftCount;
	speedRightCount = *sRightCount;

}

SpeedControl :: SpeedControl(){
 
}

SpeedControl* SpeedControl :: getInstance(){

  if(!instanca){
    instanca = true;
    instance = new SpeedControl();
  }
  return instance;
}

float SpeedControl :: racunajBrzinu(long count, long *predhodniCount, long vrem){

  float a;
  a = (count - *predhodniCount)/ (1000/vrem);
  *predhodniCount = count;
  return a;

}

void SpeedControl :: racunajVremena(){

	long vrem = millis() - predhodniMilis;

	if(vrem >= 250){

	    brzinaD = racunajBrzinu(speedRightCount,&predhodniCountD, vrem);
	    brzinaL = racunajBrzinu(speedLeftCount,&predhodniCountL, vrem);

	    predhodniMilis = milis;
  }

}

void SpeedControl :: balansirajBrzine(double *out1, double *out2){
	if(abs(brzinaL - brzinaD) >= greskaBrzine){

		if(brzinaL > brzinaD){
			*out2 += balans;
		}else{
			*out1 += balans;
		}

	}else{

	    if(brzinaL > brzinaD){
	     	*out1 -= balans;
	    }else{
	     	*out2 -= balans;
	    }

	}
}
