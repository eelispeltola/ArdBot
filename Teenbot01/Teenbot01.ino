/*
 TeenBot v1.0
 Teensy 3.2, Raspberry Pi 2 with Rasbian Jessie, 
 Actuators: two 4.5 V 48:1 DAGU motors, piezo speaker, RGB led
 Sensors: IR receiver (Sony remote controlled), Sharp IS471F
	proximity detector and Siemens SFH 205f photodiode (IR sensor)
 	with IR LEDs, two USB webcams used by RPi w/ OpenCV
 Started 16.02.2016.
 Last modified 17.02.2016.
 epe
*/

#include <RotEncoder.h>
#include <IRremote.h>
#include <Servo.h>
#include "pitches.h"

// Pin designation:
const int IRRECV_PIN = 5;

const int MOTORA_PWM_PIN = 0;	// Left motor
const int MOTORA_IN1_PIN = 0;
const int MOTORA_IN2_PIN = 0;
const int MOTORB_PWM_PIN = 0;	// Right motor
const int MOTORB_IN1_PIN = 0;
const int MOTORB_IN2_PIN = 0;

const int IRLEDL_PIN = 0;	// Left IR led TODO: which one out
const int IRLEDR_PIN = 0;	// Right IR led
const int IRSENSEL_PIN = 0;
const int IRSENSER_PIN = 0;

const int SPEAKER_PIN = 0;
const int SPEAKER_GND_PIN = 0;

const int RGB_R_PIN = 0;
const int RGB_G_PIN = 0;
const int RGB_B_PIN = 0;
const int RGB_INTS_PIN = 0;	// RGB led intensity



// Define IRremote objects
IRrecv irrecv(IRRECV_PIN);
decode_results results;

// Define encoder objects
RotEncoder leftenc(68.5, 2);  // wheel diam (mm),
							  // encoder ticks per rotation
RotEncoder rightenc(68.5, 2);  // wheel diam (mm)
							   // encoder ticks per rotation
// Variables for velocity reading
unsigned long oldMillis = 0;
float oldDist = 0;
float vel = 0;
// Variables for encoders
volatile unsigned int ticksLeft = 0;
volatile unsigned int ticksRight = 0;
unsigned int debDelay = 20;	// Debounce delay
unsigned long lastDebounceLeft = 0;
unsigned long lastDebounceRight = 0;

// Define variables for bumpers
volatile int bLeft = LOW;
volatile int bRight = LOW;

boolean started = false;

void setup() {
	// Set pin modes for encoders
	// TODO: Rewire grounds to breadboard GND
	digitalWrite(2, HIGH);    // Left enc
	digitalWrite(3, HIGH);    // Right enc

							  // Set pin modes for microswitches
	digitalWrite(leftSwitch, HIGH);
	digitalWrite(rightSwitch, HIGH);

	// Set pin modes for IR receiver and speaker
	// TODO: Grounds to breadboard GND
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	digitalWrite(6, LOW);     // IR ground
	digitalWrite(7, HIGH);     // IR Power
	pinMode(11, OUTPUT);
	digitalWrite(11, LOW);     // Speaker ground

	irrecv.enableIRIn();     // Start IR receiver

	Serial.begin(9600);

	// Set interrupts for encoders
	attachInterrupt(digitalPinToInterrupt(2), tickLeftEnc, FALLING);
	attachInterrupt(digitalPinToInterrupt(3), tickRightEnc, FALLING);

	started = true;
}

void loop() {
	if (leftSwitch == LOW) {    // If left bumper connects to GND
		bLeft == HIGH;
	}

	else if (rightSwitch == LOW) {    // If right bumper to GND
		bRight == HIGH;
	}

	if (bLeft == HIGH) {          // If left bumper hit
		Serial.println("bLeft");
		int notes[] = { NOTE_FS3, NOTE_F3, NOTE_D3 };
		int noteDurations[] = { 8,4,8 };
		reverse();
		makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
		delay(500);
		spinRight();
		delay(1500);
		forward();
		bLeft = LOW;
	}

	if (bRight == HIGH) {          // If right bumper hit
		Serial.println("bRight");
		int notes[] = { NOTE_F4, NOTE_D4, NOTE_B3 };
		int noteDurations[] = { 4,16,8 };
		reverse();
		makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
		delay(500);
		spinLeft();
		delay(1500);
		forward();
		bRight = LOW;
	}

	if (irrecv.decode(&results)) {
		switch (results.value) {
		case 0x10:
			Serial.println("1");     // Turn left, forward
			turnLeftFwd();
			break;
		case 0x810:
			Serial.println("2");     // Forward
			forward();
			break;
		case 0x410:
			Serial.println("3");     // Turn right, forward
			turnRightFwd();
			break;
		case 0xC10:
			Serial.println("4");     // Spin left
			spinLeft();
			break;
		case 0x210:
			Serial.println("5");     // Stop
			stopRobot();
			break;
		case 0xA10:
			Serial.println("6");     // Spin right
			spinRight();
			break;
		case 0x610:
			Serial.println("7");     // Turn left, reverse
			turnLeftRev();
			break;
		case 0xE10:
			Serial.println("8");     // Reverse
			reverse();
			break;
		case 0x110:
			Serial.println("9");     // Turn right, reverse
			turnRightRev();
			break;
		}
		irrecv.resume();     // Ready to receive next value
		delay(2);
	}

	// Number of rotations for both wheels
	volatile float leftrotations = leftenc.rotations(ticksLeft);
	volatile float rightrotations = rightenc.rotations(ticksRight);

	// Distance as avg of both wheels
	float dist = (leftenc.distance(clickcounts) +
		rightenc.distance(clickcounts)) / 2;

	// Velocity as avg of both wheels
	unsigned long interval = millis() - oldMillis;
	if (interval > 2000) {
		vel = (leftenc.velocity(clickcounts, interval, oldDist) +
			rightenc.velocity(clickcounts, interval, oldDist)) / 2;

		Serial.print("Rotations (left): ");
		Serial.print(leftrotations);
		Serial.print("Rotations (right): ");
		Serial.print(rightrotations);
		Serial.print(" ; Distance: ");
		Serial.print(dist);
		Serial.print(" m");
		Serial.print(" ; Speed: ");
		Serial.print(vel);
		Serial.println(" m/s");

	}
}

void makeTone(int notes[], int noteDurations[], int length) {
	// Iterate over tones
	for (int thisNote = 0; thisNote < length; thisNote++) {
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(12, notes[thisNote], noteDuration);

		// Pause to let the tone play, now with 30% extra pause
		int pauseAfterTone = noteDuration * 1.30;
		delay(pauseAfterTone);
		noTone(12);     // Stop playing tone
	}
	irrecv.enableIRIn();
	return;
}

// TODO: Add motor movement

// Interrupt handlers
void tickLeftEnc() {
	if (started && (millis() - lastDebounceLeft) > debDelay) {
		ticksLeft++;
		lastDebounceLeft = millis();
	}
}

void tickRightEnc() {
	if (started &&
		(millis() - lastDebounceRight) > debDelay) {
		ticksRight++;
		lastDebounceRight = millis();
	}
}