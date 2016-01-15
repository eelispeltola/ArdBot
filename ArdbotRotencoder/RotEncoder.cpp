/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.cpp
Takes a signal that clicks on and off repeatedly and converts it to
various RPM-type measurements.

Created 12.09.2015
Modified 16.01.2016
For Arduino Nano, ATmega328
By epe
*/

#include "RotEncoder.h"
#include "Arduino.h"


RotEncoder::RotEncoder(const float wheel_d, const int num_of_encs) {
	_circ = wheel_d * 3.1415926;  // Circumference of wheel in mm (2*pi*r = pi*d)
	_encs = num_of_encs;  // Number of encoding triggers per wheel
}


// Returns number of rotations, rounded to the nearest integer.
// TODO: Change to unsigned int for speed if too slow.
volatile float RotEncoder::rotations(volatile unsigned int clicks) {
	volatile float rotations{ clicks / _encs };
	return rotations;
}

// Returns distance in meters.
float RotEncoder::distance(volatile unsigned int clicks) {
	float dist{ _circ * rotations(clicks) * 0.001 };
	return dist;
}

// Returns velocity in (m/s).
// TODO: Test if any difference with millis() timekeeping.
// TODO: v2 with no delay(), time automatically from loop()?
float RotEncoder::velocity(volatile unsigned int clicks, unsigned long interval,
	float& oldDistance) {
	float newDist{ distance(clicks) };
	float velo{ (newDist - oldDistance) / interval };
	oldDistance = newDist;
	return velo;
}

// Clears tick count.
// TODO: Change to clear something else?? Or scrap
// void RotEncoder::clear() {
//   _tickcount = 0;
// }
