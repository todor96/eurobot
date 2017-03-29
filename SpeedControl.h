#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#define MAX_OUTPUT 100
#define MIN_OUTPUT 0

#define BROJ_MS 250

#define GRESKA_BRZINE 2
#define INC_OUTA 5

class SpeedControl {
public:

	SpeedControl(long *, long *);
    SpeedControl();

  int normalizuj(int min, int max, int a);
  
	float racunajBrzinu(long count, long *predhodniCount, long vrem);

	void racunajVremena(long speedRightCount, long speedLeftCount);

	void balansirajBrzine(double *out1, double *out2);

};

#endif
