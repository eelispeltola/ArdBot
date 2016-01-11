/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.cpp
Takes a signal that clicks on and off repeatedly and converts it to
various RPM-type measurements.

Created 12.09.2015
Modified 11.01.2016
For Arduino Nano, ATmega328
By epe
*/

#include "RotEncoder.h"
#include "Arduino.h"

namespace {

}

// ISR adds 1 to rotation count.
// TODO: Move to main program (ArdBot)
/*void RotEncoder::trigger() {
if ((millis() - _lastDebounce) > _debounce) {
_tickcount++;
_lastDebounce = millis();
}
}*/

RotEncoder::RotEncoder(const float wheel_d, const int num_of_encs) {
	//	_pin = pin;  // Input pin, 2 or 3 (ATmega328)
	_circ = wheel_d * 3.1415926;  // Circumference of wheel in mm (2*pi*r = pi*d)
	_encs = num_of_encs;  // Number of encoding triggers per wheel
						  //	_tickcount = 0;
						  //	_debounce = debounce_delay;  // Delay for debounce control in ms
						  //	_lastDebounce = 0;
}


// Returns number of rotations, rounded to the nearest integer.
// TODO: Change to word for speed if too slow.
volatile float RotEncoder::count(volatile word clicks) {
	volatile float rotations{ clicks / _encs };
	return rotations;
}

// Returns distance in meters.
float RotEncoder::distance(volatile word clicks) {
	float dist{ _circ * count(clicks) * 0.001 };
	return dist;
}

// Clears tick count.
// TODO: Change to clear something else?? Or scrap
/*void RotEncoder::clear() {
_tickcount = 0;
}*/

// Returns velocity in (m/s).
// TODO: Test if any difference with millis() timekeeping.
// TODO: v2 with no delay(), time automatically from loop()?
float RotEncoder::velocity(volatile word clicks) {
	float oldDist{ distance(clicks) };
	delay(300);
	float newDist{ distance(clicks) };
	float velo{ newDist - oldDist };  // Dividing by 1.000 s arbitrary 
	return velo;
}

