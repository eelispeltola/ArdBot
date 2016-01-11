/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.h
Takes a signal that clicks on and off repeatedly and converts it to
RPM measurements.

The circuit:
* Single reed switch connected to input 'pin'
* No outputs, switch connects to Gnd

Created 12.09.2015
Modified 04.01.2016
For Arduino Nano, ATmega328
By epe
*/

#ifndef ROTENCODER_H
#define ROTENCODER_H

#include "Arduino.h"


class RotEncoder {
	public:
		RotEncoder(const int pin, const float wheel_d, const int num_of_encs,
				const unsigned long debounce_delay);
		void setting();
		volatile word count();
		float distance();
		void clear();
		float velocity();
	private:
		int _pin;
		float _circ;
		int _encs;
		volatile word _tickcount;
		unsigned long _debounce;
		unsigned long _lastDebounce;
};

#endif // ROTENCODER_H
