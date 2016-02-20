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
 Last modified 20.02.2016.
 epe
*/

#include <RotEncoder.h>
#include <IRremote.h>
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


// Define IRremote objects
IRrecv irrecv(IRRECV_PIN);
decode_results results;

// Define encoder objects (redundant to have both, left for fluency
// in anticipation of future changes of RotEncoder class.
RotEncoder leftEnc(62, 384);  // wheel diam (mm),
							  // encoder ticks per rotation
RotEncoder rightEnc(62, 384);  // wheel diam (mm)
							   // encoder ticks per rotation

// Variables for encoder velocity reading
unsigned long oldMillisLeft = 0;
unsigned long oldMillisRight = 0;
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
byte redIntensity = 0;
byte blueIntensity = 0;
byte greenIntensity = 0;

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
	pinMode(IRRECV_PIN, INPUT);


	irrecv.enableIRIn();     // Start IR receiver

	// Set interrupts for encoders
	attachInterrupt(ENCL_IN_PIN, tickLeftEnc, CHANGE);
	attachInterrupt(ENCR_IN_PIN, tickRightEnc, CHANGE);


	started = true;
}

void loop() {

	/*lampOn(255, 0, 0);
	delay(1000);
	bankLeft(0.3, 200, 0);
	delay(1000);
	lampOn(255, 255, 255);
	bankRight(0.1, 100, 1);
	delay(1000);
	lampOff();
	forward(50);
	delay(200);
	lampOn(30, 0, 250);
	turn(90, 70, 'L');
	stop();
	lampOn(50, 150, 0);
	delay(1000);
	lampOn(0, 255, 0);
	turn(270, 100, 'R');
	lampOn(30, 30, 255);
	delay(3000);*/

	/*if (irSenseR() > 2.0) {
		bumpRight();
		Serial.println("Bumped right, Distance: ");
		Serial.println(irSenseR());
		lampOn(0, 255, 0);
	}
	else {
		lampOff();
	}
	delay(1000);*/

	/*if (bLeft == HIGH) {          // If left bumper hit
		Serial.println("Bumped left");
		int notes[] = { NOTE_FS3, NOTE_F3 };
		int noteDurations[] = { 2,3 };
		reverse(70);
		makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
		delay(500);
		turn(90, 80, 'R');
		delay(1500);
		forward(100);
		bLeft = LOW;
	}

	if (bRight == HIGH) {          // If right bumper hit
		Serial.println("Bumped right");
		int notes[] = { NOTE_F4, NOTE_D4 };
		int noteDurations[] = { 2,3 };
		reverse(70);
		makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
		delay(500);
		turn(90, 80, 'L');
		delay(1500);
		forward(100);
		bRight = LOW;
	}*/

	//::Left encoder variables with RotEncoder library::
	// Number of rotations in floating point
	volatile float rotationsLeft = leftEnc.rotations(ticksLeft);
	// Distance, in relation to wheel size.
	float distLeft = leftEnc.distance(ticksLeft);
	// Velocity, in relation to distance and interval that is
	// specified here.
	unsigned long intervalL = millis() - oldMillisLeft;
	if (intervalL > 800) {
		velLeft = leftEnc.velocity(ticksLeft, intervalL, oldDistLeft);
		oldMillisLeft = millis();
	}

	//::Right encoder variables with RotEncoder library::
	// Number of rotations in floating point
	volatile float rotationsRight = rightEnc.rotations(ticksRight);
	// Distance, in relation to wheel size. (m)
	float distRight = rightEnc.distance(ticksRight);
	// Velocity, in relation to distance and interval that is
	// specified here. (m/s)
	unsigned long intervalR = millis() - oldMillisRight;
	if (intervalR > 800) {
		velRight = rightEnc.velocity(ticksRight, intervalR, oldDistRight);
		oldMillisRight = millis();
	}

	/*Serial.print("Left	Rotations: ");
	Serial.print(rotationsLeft);
	Serial.print(" ; Distance: ");
	Serial.print(distLeft);
	Serial.print(" m");
	Serial.print(" ; Speed: ");
	Serial.print(velLeft);
	Serial.println(" m/s");
	Serial.print("Right		Rotations: ");
	Serial.print(rotationsRight);
	Serial.print(" ; Distance: ");
	Serial.print(distRight);
	Serial.print(" m");
	Serial.print(" ; Speed: ");
	Serial.print(velRight);
	Serial.println(" m/s");
	delay(10);*/


	if (irrecv.decode(&results)) {
		switch (results.value) {
		case 0x10:
			Serial.println("1");     // Turn left, forward
			bankLeft(0.1, 90, 0);
			break;
		case 0x810:
			Serial.println("2");     // Forward
			forward(90);
			break;
		case 0x410:
			Serial.println("3");     // Turn right, forward
			bankRight(0.1, 90, 0);
			break;
		case 0xC10:
			Serial.println("4");     // Spin left
			turn(90, 90, 'L');
			break;
		case 0x210:
			Serial.println("5");     // Stop
			stop();
			break;
		case 0xA10:
			Serial.println("6");     // Spin right
			turn(90, 90, 'R');
			break;
		case 0x610:
			Serial.println("7");     // Turn left, reverse
			bankLeft(0.1, 90, 1);
			break;
		case 0xE10:
			Serial.println("8");     // Reverse
			reverse(90);
			break;
		case 0x110:
			Serial.println("9");     // Turn right, reverse
			bankRight(0.1, 90, 1);
			break;
		}
		irrecv.resume();     // Ready to receive next value
		delay(2);
	}
}


// Tones for piezo speaker
void makeTone(int notes[], int noteDurations[], int length) {
	// Iterate over tones
	for (int thisNote = 0; thisNote < length; thisNote++) {
		int noteDuration = 500 / noteDurations[thisNote];
		tone(SPEAKER_PIN, notes[thisNote], noteDuration);

		// Pause to let the tone play, now with 30% extra pause
		int pauseAfterTone = noteDuration * 1.30;
		delay(pauseAfterTone);
		noTone(SPEAKER_PIN);     // Stop playing tone
	}
	irrecv.enableIRIn();
	return;
}


// Move <<motor>> with <<speed>> and <<direction>>
// Motor: A for left, B for right
// Speed: 0...255
// Direction: 0 is forward, 1 is backward
void move(char motor, byte speed, int direction) {

	digitalWrite(MOTOR_STBY_PIN, HIGH);	// Disable standby

	boolean in1PinValue = LOW;
	boolean in2PinValue = HIGH;

	if (direction == 1) {
		in1PinValue = HIGH;
		in2PinValue = LOW;
	}

	if (motor == 'A') {
		digitalWrite(MOTOR_AIN1_PIN, in1PinValue);
		digitalWrite(MOTOR_AIN2_PIN, in2PinValue);
		analogWrite(MOTOR_PWMA_PIN, speed);
	}
	else {
		digitalWrite(MOTOR_BIN1_PIN, in1PinValue);
		digitalWrite(MOTOR_BIN2_PIN, in2PinValue);
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

// TODO: integrate speed sensors to bank with existing speed?
// Turns right as it moves in <<direction>> with <<speed>>
// Amount: 0...1, specifies sharpness of turning
void bankRight(float amount, byte speed, int direction) {
	byte Bspeed = byte((float)speed * amount);
	move('A', speed, direction);
	move('B', Bspeed, direction);
	Serial.println("Banking right");
}

// Turns left as it moves in <<direction>> with <<speed>>
// Amount: 0...1, specifies sharpness of turning
void bankLeft(float amount, byte speed, int direction) {
	byte Aspeed = byte((float)speed * amount);
	move('B', speed, direction);
	move('A', Aspeed, direction);
	Serial.println("Banking left");
}

// TODO: Bump sensing while turning
// Turns <<degrees>> with <<speed>> in <<direction>> on the spot.
// <<direction>> is 'l' or 'L' for left, or 'r' or 'R' for right.
void turn(float degrees, byte speed, char direction) {
	float axle = 155;	// Length of line between wheels in mm
	float distToTurn = axle * (degrees / 360.0) * 3.1416;
	unsigned int oldTicksLeft = ticksLeft;
	volatile float distTurned = leftEnc.distance(ticksLeft - oldTicksLeft)/1000.0;
	int dirA = 0;
	int dirB = 1;

	if (direction == 'l' || direction == 'L') {
		dirA = 1;
		dirB = 0;
	}

	Serial.println("Turning");

	// 0.3 overhead when turning because of motor coasting
	while (distTurned + 0.3 < distToTurn) {
		if (bLeft || bRight) {
			break;
		}
		move('A', speed, dirA);
		move('B', speed, dirB);
		distTurned = leftEnc.distance(ticksLeft - oldTicksLeft)*1000;
	}
	stop();
	/*Serial.print("Turning: direction; degrees; speed; distToTurn		");
	Serial.print(direction);
	Serial.print("; ");
	Serial.print(degrees);
	Serial.print("; ");
	Serial.print(speed);
	Serial.print("; ");
	Serial.print(distToTurn);
	Serial.print("	DistTurned: ");
	Serial.println(distTurned);*/
}

// Changes global RGB intensity values and passes them to RGB led
// 0 is off, 255 is max for each color
void lampOn(unsigned int red, unsigned int green, unsigned int blue) {
	redIntensity = red;
	greenIntensity = green;
	blueIntensity = blue;
	analogWrite(RGB_R_PIN, redIntensity);
	analogWrite(RGB_G_PIN, greenIntensity);
	analogWrite(RGB_B_PIN, blueIntensity);
}

// Turns RGB led off
void lampOff() {
	analogWrite(RGB_R_PIN, 0);
	analogWrite(RGB_G_PIN, 0);
	analogWrite(RGB_B_PIN, 0);
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
