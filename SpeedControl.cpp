#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "SpeedControl.h"
#include <Math.h>

long predhodniMilis = 0;

long predhodniCountD = 0;
long predhodniCountL = 0;

float brzinaD = 0;
float brzinaL = 0;

SpeedControl :: SpeedControl(){
 
}

float SpeedControl :: racunajBrzinu(long count, long *predhodniCount, long vrem){      //racuna brzine preko otkucaja enkodera i vremena izmedju dva otkucaja enkodera
																					   //tacnije broj obrtaja enkodera po sekundi nam je brzina
  float a;
  a = float(count - *predhodniCount) / (1000.0/vrem);
  *predhodniCount = count;
  return a;

}

void SpeedControl :: racunajVremena(long speedRightCount, long speedLeftCount){

	long milis = millis();
	long vrem = milis - predhodniMilis;

	if(vrem >= BROJ_MS){

	    brzinaD = racunajBrzinu(speedRightCount,&predhodniCountD, vrem);
	    brzinaL = racunajBrzinu(speedLeftCount,&predhodniCountL, vrem);

	    predhodniMilis = milis;
  }

}

void SpeedControl :: balansirajBrzine(double *out1, double *out2){
	if(abs(brzinaL - brzinaD) >= GRESKA_BRZINE){

		if(brzinaL > brzinaD){
			*out2 += INC_OUTA;
		}else{
			*out1 += INC_OUTA;
		}

		*out1 = normalizuj(MIN_OUTPUT, MAX_OUTPUT, *out1);
		*out2 = normalizuj(MIN_OUTPUT, MAX_OUTPUT, *out2);

	}else{

	    if(brzinaL > brzinaD){
	     	*out1 -= INC_OUTA;
	    }else{
	     	*out2 -= INC_OUTA;
	    }

	    *out1 = normalizuj(MIN_OUTPUT, MAX_OUTPUT, *out1);
		*out2 = normalizuj(MIN_OUTPUT, MAX_OUTPUT, *out2);
	    
	}
}

int SpeedControl :: normalizuj(int min, int max, int a){
	if(a > max){
		a=max;
	}else if(a < min){
		a=min;
	}

	return a;
}
