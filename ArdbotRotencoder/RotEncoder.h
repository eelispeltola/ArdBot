/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.h
Takes a signal that clicks on and off repeatedly and converts it to
various RPM-type measurements.

Created 12.09.2015
Modified 11.01.2016
For Arduino Nano, ATmega328
By epe
*/

#ifndef ROTENCODER_H
#define ROTENCODER_H

#include "Arduino.h"

// TODO: Clear up when tested in code
class RotEncoder {
public:
	RotEncoder(const float wheel_d, const int num_of_encs);
	volatile float count(volatile word clicks);
	float distance(volatile word clicks);
	//		void clear();
	float velocity(volatile word clicks);
	//		void trigger();
private:
	//		int _pin;
	float _circ;
	int _encs;
	//		volatile word _tickcount;
	//		unsigned long _debounce;
	//		unsigned long _lastDebounce;
};

#endif // ROTENCODER_H
