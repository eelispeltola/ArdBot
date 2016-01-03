/*
ArdBot Rotation Encoder

Module: rotary encoder / File: RotEncoder.cpp
Takes a signal that clicks on and off repeatedly and converts it to
RPM measurements.

The circuit:
* Single reed switch connected to input 'pin'
* No outputs, switch connects to Gnd

Created 12.09.2015
Modified 03.01.2016
For Arduino Nano, ATmega328
By epe
*/

#include "RotEncoder.h"
#include "Arduino.h"

namespace {
	// ISR adds 1 to rotation count.
	void trigger() {
		if ((millis() - lastDebounce) > _debounce) {
			_tickcount++;
			_lastDebounce = millis();
		}
	}
}

Encoder::Encoder(const int pin, const float wheel_d, const int num_of_encs,
				 const unsigned long debounce_delay) {
	_pin = pin;  // Input pin, 2 or 3 (ATmega328)
	_circ = wheel_d * 3.1415926;  // Circumference of wheel in mm (2*pi*r = pi*d)
	_encs = num_of_encs;  // Number of encoding triggers per wheel
	_tickcount = 0;
	_debounce = debounce_delay;  // Delay for debounce control
	_lastDebounce = 0;
}

// Run at setup.
void Encoder::setting() {
	digitalWrite(_pin, HIGH);
	// Attaches interrupt 0 or 1 (pin 2 or 3, ATmega328) to trigger() when falling
	attachInterrupt(digitalPinToInterrupt(_pin), trigger, FALLING);
}

// Returns number of rotations, rounded to the nearest integer.
// TODO: Change to float for precision, if little speed lost.
volatile word Encoder::count() {
	rotations  = _tickcount / _encs;
	return rotations;
}

// Returns distance in meters.
float Encoder::distance() {
	dist = _circ * count() * 0.001;
	return dist;
}

// Clears tick count.
void Encoder::clear() {
	_tickcount = 0;
}

// Returns velocity in (m/s).
// TODO: Test if any difference with millis() timekeeping.
// TODO: v2 with no delay(), time automatically from loop().
float Encoder::velocity() {
	float oldDist = distance();
	delay(300);
	float newDist = distance();
	velo = newDist - oldDist;  // Dividing by 1.000 s arbitrary 
	return velo;
}

