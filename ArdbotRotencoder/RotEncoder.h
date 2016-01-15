/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.h
Takes a signal that clicks on and off repeatedly and converts it to
various RPM-type measurements.

Created 12.09.2015
Modified 16.01.2016
For Arduino Nano, ATmega328
By epe
*/

#ifndef ROTENCODER_H
#define ROTENCODER_H


#include "Arduino.h"

class RotEncoder {
public:
	RotEncoder(const float wheel_d, const int num_of_encs);
	volatile float rotations(volatile unsigned int clicks);
	float distance(volatile unsigned int clicks);
	float velocity(volatile unsigned int clicks, unsigned long interval,
		float& oldDistance);
private:
	float _circ;
	int _encs;
};


#endif // ROTENCODER_H
