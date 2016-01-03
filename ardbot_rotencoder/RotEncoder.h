/*
ArdBot Rotation Encoder

Module: encoder / File: encoder.h
Takes a signal that clicks on repeatedly and converts it to
RPM measurements.

The circuit:
* Single reed switch connected to input 'pin'
* No outputs, switch connects to Gnd

Created 12.09.2015
Modified 03.01.2016
For Arduino Nano, ATmega328
By epe
*/

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"


class Encoder {
	public:
		Encoder(const int pin, const float wheel_d, const int num_of_encs);
		void setting();
		volatile word count();
		float distance();
		void clear();
		float velocity(const unsigned long milliseconds);
	private:
		int _pin;
		int _circ;
		int _encs;
		volatile word _clickcount;
};

#endif // ENCODER_H
