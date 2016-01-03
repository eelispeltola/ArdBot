/*
ArdBot Rotation Encoder

Module: encoder / File: encoder.cpp
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

#include "RotEncoder.h"
#include "Arduino.h"


Encoder::Encoder(const int pin, const float wheel_d, const int num_of_encs) {
	_pin = pin;  // Input pin, 2 or 3 (ATmega328)
	_circ = wheel_d * 3.1415926;  // Circumference of wheel in mm (2*pi*r = pi*d)
	_encs = num_of_encs;  // Number of encoding triggers per wheel
	_clickcount = 0;
}

// Run at setup.
void setting() {
	digitalWrite(_pin, HIGH);
	// Attaches interrupt 0 or 1 (pin 2 or 3, ATmega328) to trigger() when falling
	attachInterrupt(digitalPinToInterrupt(_pin), trigger, FALLING);
}

// ISR adds 1 to rotation count.
void trigger() {
	_clickcount++;
}

// Returns number of rotations, rounded to the nearest integer.
// TODO: Change to float for precision, if little speed lost.
volatile word Encoder::count() {
	rotations  = _clickcount / _encs;
	return rotations;
}

// Returns distance in meters.
float Encoder::distance() {
	dist = _circ * count() * 0.001;
	return dist;
}

// Clears click count.
void Encoder::clear() {
	_clickcount = 0;
}

// Returns velocity in (m/s)
float Encoder::velocity(const unsigned long milliseconds) {
	velo = distance() / ((float)milliseconds * 0.001);
	return velo;
}
