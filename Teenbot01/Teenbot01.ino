/*
 TeenBot v1.0
 Teensy 3.2, Raspberry Pi 2 with Rasbian Jessie, SF TB6612FNG
 motor driver breakout
 Actuators: two 4.5 V 48:1 DAGU motors, piezo speaker, RGB led
 Sensors: IR receiver (Sony remote controlled), two DAGU
	hall effect encoders, Sharp IS471F proximity detector
	and Siemens SFH 205f photodiode (IR recv) with IR LEDs,
	two USB webcams used by RPi w/ OpenCV
 Started 16.02.2016.
 Last modified 17.02.2016.
 epe
*/

#include <RotEncoder.h>
// #include <IRremote.h>	TODO: FIND ANOTHER LIBRARY
// (IRremote not working on Teensy)
#include "pitches.h"

// Pin designation:
const int IRRECV_PWR_PIN = 13;	// IR (remote) receiver power
const int IRRECV_GND_PIN = 14;	// IR receiver ground
const int IRRECV_PIN = 15;	// IR receiver data in

const int MOTOR_PWMA_PIN = 3;	// Left motor
const int MOTOR_AIN2_PIN = 4;
const int MOTOR_AIN1_PIN = 5;
const int MOTOR_STBY_PIN = 6;	// Standby
const int MOTOR_BIN1_PIN = 7;	// Right motor
const int MOTOR_BIN2_PIN = 8;
const int MOTOR_PWMB_PIN = 9;

const int SPEAKER_PIN = 0;	// Piezo speaker

const int IRLEDR_PIN = 23;	// Right IR led
const int IRSENSEL_PIN = A5;	// IS471F, left detector data in
const int IRSENSER_PIN = A4;	// SHF 205f, right detector data in

const int ENCL_IN_PIN = 1;	// Left encoder data in 
const int ENCR_IN_PIN = 17;

const int RGB_R_PIN = A8;	// Common cathode RGB led
const int RGB_G_PIN = A7;
const int RGB_B_PIN = A6;


// Define IRremote objects (IRremote not working on Teensy)
//IRrecv irrecv(IRRECV_PIN);
//decode_results results;

// Define encoder objects
RotEncoder leftEnc(62, 384);  // wheel diam (mm),
							  // encoder ticks per rotation
RotEncoder rightEnc(62, 384);  // wheel diam (mm)
							   // encoder ticks per rotation
// Variables for encoder velocity reading
unsigned long oldMillis = 0;
float oldDistLeft = 0;
float oldDistRight = 0;
float velLeft = 0;
float velRight = 0;
// Variables for encoder interrupts and debouncing
volatile unsigned int ticksLeft = 0;
volatile unsigned int ticksRight = 0;
unsigned int debDelay = 1;	// Debounce delay
unsigned long lastDebounceLeft = 0;
unsigned long lastDebounceRight = 0;

// Define variables for bumping
volatile boolean bLeft = false;
volatile boolean bRight = false;

// Define led intensities
unsigned int redIntensity = 0;
unsigned int blueIntensity = 0;
unsigned int greenIntensity = 0;

boolean started = false;

void setup() {
	Serial.begin(9600);

	pinMode(RGB_R_PIN, OUTPUT);
	pinMode(RGB_G_PIN, OUTPUT);
	pinMode(RGB_B_PIN, OUTPUT);

	pinMode(MOTOR_STBY_PIN, OUTPUT);
	pinMode(MOTOR_PWMA_PIN, OUTPUT);
	pinMode(MOTOR_AIN1_PIN, OUTPUT);
	pinMode(MOTOR_AIN2_PIN, OUTPUT);
	pinMode(MOTOR_PWMB_PIN, OUTPUT);
	pinMode(MOTOR_BIN1_PIN, OUTPUT);
	pinMode(MOTOR_BIN2_PIN, OUTPUT);

	pinMode(ENCL_IN_PIN, INPUT_PULLUP);
	pinMode(ENCR_IN_PIN, INPUT_PULLUP);

	pinMode(IRRECV_PWR_PIN, OUTPUT);	// IR recv power
	digitalWrite(IRRECV_PWR_PIN, HIGH);
	pinMode(IRRECV_GND_PIN, OUTPUT);	// IR recv ground
	digitalWrite(IRRECV_GND_PIN, LOW);
	// pinMode(IRRECV_PIN, INPUT);	// IR recv data in TODO: AWAY FOR GOOD??

	//irrecv.enableIRIn();     // Start IR receiver

	// Set interrupts for encoders
	attachInterrupt(ENCL_IN_PIN, tickLeftEnc, CHANGE);
	attachInterrupt(ENCR_IN_PIN, tickRightEnc, CHANGE);

	started = true;
}

void loop() {

	forward(100);
	delay(200);
	reverse(80);
	delay(100);
	bankLeft(0.7, 255, 0);
	delay(150);
	bankRight(0.1, 100, 1);
	delay(400);
	turnLeft(90, 150);
	turnRight(270, 30);

	if (bLeft == HIGH) {          // If left bumper hit
		Serial.println("bLeft");
		int notes[] = { NOTE_FS3, NOTE_F3 };
		int noteDurations[] = { 2,3 };
		reverse(70);
		makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
		delay(500);
		turnRight(90, 51);
		delay(1500);
		forward(100);
		bLeft = LOW;
	}

	if (bRight == HIGH) {          // If right bumper hit
		Serial.println("bRight");
		int notes[] = { NOTE_F4, NOTE_D4 };
		int noteDurations[] = { 2,3 };
		reverse(70);
		makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
		delay(500);
		turnLeft(90, 51);
		delay(1500);
		forward(100);
		bRight = LOW;
	}

/*	if (irrecv.decode(&results)) {
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
	}*/

	//::Left encoder variables with RotEncoder library::
	// Number of rotations in floating point
	volatile float rotationsLeft = leftEnc.rotations(ticksLeft);
	// Distance, in relation to wheel size.
	float distLeft = leftEnc.distance(ticksLeft);
	// Velocity, in relation to distance and interval that is
	// specified here.
	unsigned long intervalL = millis() - oldMillis;
	if (intervalL > 400) {
		velLeft = leftEnc.velocity(ticksLeft, intervalL, oldDistLeft);
		oldMillis = millis();
	}

	//::Right encoder variables with RotEncoder library::
	// Number of rotations in floating point
	volatile float rotationsRight = rightEnc.rotations(ticksRight);
	// Distance, in relation to wheel size. (m)
	float distRight = rightEnc.distance(ticksRight);
	// Velocity, in relation to distance and interval that is
	// specified here. (m/s)
	unsigned long intervalR = millis() - oldMillis;
	if (intervalR > 400) {
		velRight = rightEnc.velocity(ticksRight, intervalR, oldDistRight);
		oldMillis = millis();
	}

	Serial.print("Left		Rotations: ");
	Serial.print(rotationsLeft);
	Serial.print(" ; Distance: ");
	Serial.print(distLeft);
	Serial.print(" m");
	Serial.print(" ; Speed: ");
	Serial.print(velLeft);
	Serial.print(" m/s	||		");
	Serial.print("Right		Rotations: ");
	Serial.print(rotationsRight);
	Serial.print(" ; Distance: ");
	Serial.print(distRight);
	Serial.print(" m");
	Serial.print(" ; Speed: ");
	Serial.print(velRight);
	Serial.println(" m/s");

}


// Tones for piezo speaker
void makeTone(int notes[], int noteDurations[], int length) {
	// Iterate over tones
	for (int thisNote = 0; thisNote < length; thisNote++) {
		int noteDuration = 500 / noteDurations[thisNote];
		tone(12, notes[thisNote], noteDuration);

		// Pause to let the tone play, now with 30% extra pause
		int pauseAfterTone = noteDuration * 1.30;
		delay(pauseAfterTone);
		noTone(12);     // Stop playing tone
	}
	// irrecv.enableIRIn();
	return;
}


// Move <<motor>> with <<speed>> and <<direction>>
//Motor: A for left, B for right
//Speed: 0...255
//direction: 0 is forward, 1 is backward
void move(char motor, byte speed, int direction) {
	//Motor: A for left, B for right
	//Speed: 0...255
	//direction: 0 is forward, 1 is backward

	digitalWrite(MOTOR_STBY_PIN, HIGH); //disable standby

	boolean inPin1 = LOW;
	boolean inPin2 = HIGH;

	if (direction == 1) {
		inPin1 = HIGH;
		inPin2 = LOW;
	}

	if (motor == 'A') {
		digitalWrite(MOTOR_AIN1_PIN, inPin2);
		digitalWrite(MOTOR_AIN2_PIN, inPin1);
		analogWrite(MOTOR_PWMA_PIN, speed);
	}
	else {
		digitalWrite(MOTOR_BIN1_PIN, inPin1);
		digitalWrite(MOTOR_BIN2_PIN, inPin2);
		analogWrite(MOTOR_PWMB_PIN, speed);
	}
}

// Stop motors by enabling standby
void stop() {
	digitalWrite(MOTOR_STBY_PIN, LOW);
}

// TODO: integrate speed sensors for master/slave motors
void forward(byte speed) {
	move('A', speed, 0);
	move('B', speed, 0);
}

void reverse(byte speed) {
	move('A', speed, 1);
	move('B', speed, 1);
}

// TODO: integrate speed sensors to bank with existing speed.
// Turns right as it moves in <<direction>> with <<speed>>
// Amount: 0...1, specifies sharpness of turning
void bankRight(float amount, byte speed, int direction) {
	byte Bspeed = byte((float)speed * amount);
	move('A', speed, direction);
	move('B', Bspeed, direction);
}

// Turns right as it moves in <<direction>> with <<speed>>
// Amount: 0...1, specifies sharpness of turning
void bankLeft(float amount, byte speed, int direction) {
	byte Aspeed = byte((float)speed * amount);
	move('B', speed, direction);
	move('A', Aspeed, direction);
}

// TODO: Bump sensing while turning
// Turns left <<degrees>> with <<speed>> on the spot
void turnLeft(int degrees, byte speed) {
	unsigned int axle = 0;	// Length of line between wheels in mm
	unsigned int distToTurn = axle * (degrees / 360) * 3.1416;
	unsigned int oldTicksLeft = ticksLeft;
	volatile float distTurned = leftEnc.distance(ticksLeft - oldTicksLeft)/1000;
	while (distTurned <= distToTurn) {
		move('A', speed, 0);
		move('B', speed, 1);
		distTurned = leftEnc.distance(ticksLeft - oldTicksLeft)/1000;
		delay(2);
	}
}

// Turns right <<degrees>> with <<speed>> on the spot
void turnRight(int degrees, byte speed) {
	unsigned int axle = 0;	// Length of line between wheels in mm
	unsigned int distToTurn = axle * (degrees / 360) * 3.1416;
	unsigned int oldTicksLeft = ticksLeft;
	volatile float distTurned = leftEnc.distance(ticksLeft - oldTicksLeft) / 1000;
	while (distTurned <= distToTurn) {
		move('A', speed, 1);
		move('B', speed, 0);
		distTurned = leftEnc.distance(ticksLeft - oldTicksLeft) / 1000;
	}
}

//:::Interrupt handlers:::::::::::::::::::::::::::::::::::::::

void tickLeftEnc() {
	if (started && (millis() - lastDebounceLeft) > debDelay) {
		ticksLeft++;
		lastDebounceLeft = millis();
	}
}

void tickRightEnc() {
	if (started && (millis() - lastDebounceRight) > debDelay) {
		ticksRight++;
		lastDebounceRight = millis();
	}
}

void bumpLeft() {
	if (started) {
		bLeft = true;
	}
}

void bumpRight() {
	if (started) {
		bRight = true;
	}
}
