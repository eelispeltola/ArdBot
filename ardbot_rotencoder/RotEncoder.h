/*
ArdBot RPM encoder

Module: encoder / File: encoder.h
Takes a signal that clicks on repeatedly and converts it to
RPM measurements.

The circuit:
* these attached to input this and this
* these attached to output this and this

Created 12.09.2015
Modified 29.10.2015
For Arduino Nano, ATMega328
By epe
*/

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"


class Encoder {
public:
	Encoder(int pin, int wheel_d, int num_of_encs);
	uint distance();
private:
	int _pin;
	int _diam;
	int _encs;
};

#endif // ENCODER_H