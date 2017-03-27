#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif


class SpeedControl {
public:

	SpeedControl(long *, long *);
  SpeedControl();

  SpeedControl* SpeedControl :: getInstance();

	float SpeedControl :: racunajBrzinu(long count, long *predhodniCount, long vrem);

	void SpeedControl :: racunajVremena();

	void SpeedControl :: balansirajBrzine(double *out1, double *out2);

};

#endif
