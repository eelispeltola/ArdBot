/*
ArdBot RPM encoder

Module: encoder / File: encoder.cpp
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

#include "encoder.h"
#include "Arduino.h"


Encoder::Encoder(int pin, int wheel_d, int num_of_encs) {
	_pin = pin;
	_diam = wheel_d;
	_encs = num_of_encs;
	volatile int click = LOW;
	digitalWrite(_pin, HIGH);
	attachInterrupt(_pin, click = HIGH, CHANGE);
}

uint Encoder::distance() {

}