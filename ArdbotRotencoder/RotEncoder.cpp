/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.cpp
Takes a signal that clicks on and off repeatedly and converts it to
various RPM-type measurements.

Created 12.09.2015
Modified 20.02.2016
For Arduino Nano, ATmega328
By epe
*/

#include "RotEncoder.h"
#include "Arduino.h"


RotEncoder::RotEncoder(const float wheel_d, const int num_of_encs) {
	_circ = wheel_d * 3.1415926;  // Circumference of wheel in mm (2*pi*r = pi*d)
	_encs = num_of_encs;  // Number of encoding triggers per wheel
}


// Returns number of rotations.
// TODO: Change to unsigned int for speed if too slow.
volatile float RotEncoder::rotations(volatile unsigned int clicks) {
	volatile float rotations{ (float)clicks / (float)_encs };
	return rotations;
}

// Returns distance in meters.
float RotEncoder::distance(volatile unsigned int clicks) {
	float dist{ _circ * rotations(clicks) * 0.001 };
	return dist;
}

// Returns velocity in (m/s). Requires main loop to run faster than <<interval>>.
// Pass <<oldDistance>> as meters and interval as milliseconds.
float RotEncoder::velocity(volatile unsigned int clicks, unsigned long interval,
						   float& oldDistance) {
	float newDist{ distance(clicks) };
	float velo{ (newDist - oldDistance)*1000 / interval };
	oldDistance = newDist;
	return velo;
}

// Clears tick count.
// TODO: Scrap? Kept for future modifications
// void RotEncoder::clear() {
//   _tickcount = 0;
// }
