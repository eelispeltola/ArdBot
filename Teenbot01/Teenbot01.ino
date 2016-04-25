/*
 TeenBot v1.0
 Teensy 3.2, Raspberry Pi 2 with Rasbian Jessie, SF TB6612FNG
 motor driver breakout
 Actuators: two 4.5 V 48:1 DAGU motors, piezo speaker, RGB led
 Sensors: IR receiver (Sony remote controlled), two DAGU
	hall effect encoders, Sharp IS471F proximity detector
	and Siemens SFH 205f photodiode (IR recv) with IR LEDs,
	two USB webcams used by RPi w/ OpenCV
 Started: 16.02.2016.
 Last modified: 25.04.2016.
 Author: Eelis Peltola
*/

#include <RotEncoder.h>
#include <IRremote.h>
#include "pitches.h"

// Pin designation:
#define IRRECV_PWR_PIN 13	// IR (remote) receiver power
#define IRRECV_GND_PIN 14	// IR receiver ground
#define IRRECV_PIN 15	// IR receiver data in

#define MOTOR_PWMA_PIN 3	// Left motor
#define MOTOR_AIN2_PIN 4
#define MOTOR_AIN1_PIN 5
#define MOTOR_STBY_PIN 6	// Standby
#define MOTOR_BIN1_PIN 7	// Right motor
#define MOTOR_BIN2_PIN 8
#define MOTOR_PWMB_PIN 9

#define SPEAKER_PIN 0	// Piezo speaker

#define IRLEDR_PIN 23	// Right IR led
#define IRSENSEL_PIN A5	// IS471F, left detector data in
#define IRSENSER_PIN A4	// SHF 205f, right detector data in

#define ENCL_IN_PIN 1	// Left encoder data in 
#define ENCR_IN_PIN 17

#define RGB_R_PIN A8	// Common cathode RGB led
#define RGB_G_PIN A7
#define RGB_B_PIN A6

#define DEBUG	// Comment out to omit debug messages


// <<DEBUG_PRINT(str)>> accompanies data for a given string
#ifdef DEBUG
#define DEBUG_PRINT(str) \
	Serial.print(millis()); \
	Serial.print(": "); \
	Serial.print(__FUNCTION__); \
	Serial.print("() in "); \
	Serial.print(__FILE__); \
	Serial.print(':'); \
	Serial.print(__LINE__); \
	Serial.print(' '); \
	Serial.println(str);
#endif // !DEBUG

// Define IRremote objects
IRrecv irrecv(IRRECV_PIN);
decode_results results;

// Define encoder objects (redundant to have both, left for fluency
// in anticipation of future changes to RotEncoder class)
RotEncoder leftEnc(62, 384);  // wheel diam (mm),
							  // encoder ticks per rotation
RotEncoder rightEnc(62, 384);  // wheel diam (mm)
							   // encoder ticks per rotation

// Variables for encoder velocity reading
unsigned long oldMillisLeft = 0;
unsigned long oldMillisRight = 0;
float oldDistLeft = 0;
float oldDistRight = 0;

// Variables for encoder interrupts and debouncing
volatile unsigned int ticksLeft = 0;
volatile unsigned int ticksRight = 0;
unsigned int debDelay = 1;	// Debounce delay
unsigned long lastDebounceLeft = 0;
unsigned long lastDebounceRight = 0;

// Define variables for bumping
volatile boolean bLeft = false;
volatile boolean bRight = false;


// Define variables for turning based on RPi commands
int rpiTurnAmount = 0;
int rpiTurnedAmount = 0;
bool objInSight;

// Define led intensities
byte redIntensity = 0;
byte blueIntensity = 0;
byte greenIntensity = 0;

boolean started = false;

void setup() {
	Serial.begin(9600);

	// RGB led
	pinMode(RGB_R_PIN, OUTPUT);
	pinMode(RGB_G_PIN, OUTPUT);
	pinMode(RGB_B_PIN, OUTPUT);
	
	// Motor driver
	pinMode(MOTOR_STBY_PIN, OUTPUT);
	pinMode(MOTOR_PWMA_PIN, OUTPUT);
	pinMode(MOTOR_AIN1_PIN, OUTPUT);
	pinMode(MOTOR_AIN2_PIN, OUTPUT);
	pinMode(MOTOR_PWMB_PIN, OUTPUT);
	pinMode(MOTOR_BIN1_PIN, OUTPUT);
	pinMode(MOTOR_BIN2_PIN, OUTPUT);

	// Encoding
	pinMode(ENCL_IN_PIN, INPUT_PULLUP);
	pinMode(ENCR_IN_PIN, INPUT_PULLUP);

	// IR receiver
	pinMode(IRRECV_PWR_PIN, OUTPUT);	// IR recv power
	digitalWrite(IRRECV_PWR_PIN, HIGH);
	pinMode(IRRECV_GND_PIN, OUTPUT);	// IR recv ground
	digitalWrite(IRRECV_GND_PIN, LOW);
	pinMode(IRRECV_PIN, INPUT);

	// IR obstacle sensing
	pinMode(IRLEDR_PIN, OUTPUT);
	pinMode(IRSENSEL_PIN, INPUT);
	pinMode(IRSENSER_PIN, INPUT);

	irrecv.enableIRIn();     // Start IR receiver

	// Set interrupts for encoders
	attachInterrupt(ENCL_IN_PIN, tickLeftEnc, CHANGE);
	attachInterrupt(ENCR_IN_PIN, tickRightEnc, CHANGE);

	// Set interrupt for Sharp IS471f IR proximity sensor
	attachInterrupt(IRSENSEL_PIN, bumpLeft, FALLING);

	started = true;
	lampOn(255, 0, 0);
	delay(1500);
	lampOn(255, 0, 255);

}

void loop() {

	if (irSenseR() > 2.0) {		// If obstacle at right sensor
		bumpRight();
		DEBUG_PRINT("irSenseRBump");
	#ifdef DEBUG
	Serial.println("Bumped right, Distance: ");
	Serial.println(irSenseR());
	#endif // !DEBUG

	}

	
	if (bLeft == HIGH) {          // If left bumper hit
		bLeft = LOW;
		bumpMovement('L');
	}

	if (bRight == HIGH) {          // If right bumper hit
		bRight = LOW;
		bumpMovement('R');
	}

	
	// Check serial bus for messages. A number more than +-100
	// means no object is in sight, lower is amount that must be
	// turned (-100...100).
	// TODO: Proper handling of messages, conversion byte->int
	if (Serial.available()) {
		int rpiMessage = (int)Serial.read();
		if (abs(rpiMessage) > 100) {
			objInSight == false;
		}
		else {
			rpiTurnAmount = rpiMessage;
		}
	}


	// Turn based on RPi Serial commands.
	// Both <<rpiTurnedAmount>> and <<rpiTurnAmount>>
	// origos are at center of robot -> neg. values turn right
	if (objInSight && !bLeft && !bRight) {
		int amountToTurn = rpiTurnedAmount - rpiTurnAmount;
		if (amountToTurn > 5) {
			bankLeft(0.2, 90, 0);
		}
		if (amountToTurn < 5) {
			bankRight(0.2, 90, 0);
		}
		else {
			forward(90);
		}
	}

	// If IR Receiver commands are available,
	// move based on them.
	if (irrecv.decode(&results)) {
		receiverMovement(results);
	}
}


// Switch direction based on IR Receiver commands
void receiverMovement(decode_results results) {
	switch (results.value) {
	case 0x10:
		DEBUG_PRINT("1");
		bankLeft(0.2, 90, 0);     // Turn left, forward
		break;
	case 0x810:
		DEBUG_PRINT("2");
		forward(90);     // Move forward
		break;
	case 0x410:
		DEBUG_PRINT("3");
		bankRight(0.2, 90, 0);     // Turn right, forward
		break;
	case 0xC10:
		DEBUG_PRINT("4");
		turn(90, 90, 'L');     // Spin left
		break;
	case 0x210:
		DEBUG_PRINT("5");
		stop();    // Stop
		break;
	case 0xA10:
		DEBUG_PRINT("6");
		turn(90, 90, 'R');     // Spin right
		break;
	case 0x610:
		DEBUG_PRINT("7");
		bankLeft(0.2, 90, 1);     // Turn left, reverse
		break;
	case 0xE10:
		DEBUG_PRINT("8");
		reverse(90);     // Reverse
		break;
	case 0x110:
		DEBUG_PRINT("9");
		bankRight(0.2, 90, 1);     // Turn right, reverse
		break;
	}
	irrecv.resume();     // Ready to receive next value
	delay(2);
}



// Measures presence of UV light for one ms, outputs unitless
// distance-to-obstacle value. Takes ambient UV light into account.
float irSenseR() {
	int halfPeriod = 30;  // One cycle in microseconds, divided in two
	const int cycles = (int)(1 / (float(halfPeriod) * 2) * 1000);	// Cycles in 1 ms
	int ambient = 0;
	int illuminated = 0;
	int distanceToObstacle = 0;
	int irValues[cycles];
	int i;

	// Flashes IR led on/off for <<cycles>>, measuring ambient UV
	// and UV bounced from led each time.
	for (i = 0; i < cycles; i++) {
		digitalWrite(IRLEDR_PIN, LOW);
		delayMicroseconds(halfPeriod);
		ambient = analogRead(IRSENSER_PIN);
		digitalWrite(IRLEDR_PIN, HIGH);
		delayMicroseconds(halfPeriod);
		illuminated = analogRead(IRSENSER_PIN);
		// Photodiode resistance grows with presence of UV light -> illuminated
		// values are lower.
		irValues[i] = ambient - illuminated;
		distanceToObstacle += irValues[i];
	}

	#ifdef DEBUG
	DEBUG_PRINT("IR LED");
	Serial.print("Right IR values: ");
	float distancetotarget = 0;
	for (i = 0; i < cycles; i++) {
	distancetotarget += irValues[i];
	Serial.print(irValues[i]);
	Serial.print(";");
	}
	Serial.println("");
	#endif // DEBUG
	

	// Distance is given as average of distances for the whole millisecond.
	float avgDistance = distanceToObstacle / (float)cycles;
	avgDistance = max(avgDistance, 0.0);

	return avgDistance;
}


// TODO: Replace delay with global timekeeping
// Reverse and turn after a bumper hit.
// <<bumpDir>> is 'L' or 'R' for side of bumper.
void bumpMovement(char bumpDir) {
	char turnDir;
	int notes[2];
	if (bumpDir == 'L') {
		lampOn(0, 255, 0);
		notes[0] = NOTE_FS3;
		notes[1] = NOTE_F3;
		turnDir = 'R';
	}
	else if (bumpDir == 'R') {
		lampOn(0, 0, 255);
		notes[0] = NOTE_F4;
		notes[1] = NOTE_D4;
		turnDir = 'L';
	}
	else {
		return;
	}

	#ifdef DEBUG
	Serial.print("Bumped ");
	Serial.println(bumpDir);
	#endif // DEBUG
	
	int noteDurations[] = { 2,3 };
	reverse(140);
	makeTone(notes, noteDurations, sizeof(notes) / sizeof(int));
	delay(500);
	turn(45, 200, turnDir);
	forward(170);
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
//		Or take existing speed from global variable.
// Turns right as it moves in <<direction>> with <<speed>>
// Amount: 0...1 specifies sharpness of turning, with
// 1 being no turn at all and 0 a sharp turn.
void bankRight(float amount, byte speed, int direction) {
	byte Bspeed = byte((float)speed * (1 - amount));
	move('A', speed, direction);
	move('B', Bspeed, direction);
	DEBUG_PRINT("Banking right");
}

// Turns left as it moves in <<direction>> with <<speed>>
// Amount: 0...1 specifies sharpness of turning
void bankLeft(float amount, byte speed, int direction) {
	byte Aspeed = byte((float)speed * (1 - amount));
	move('B', speed, direction);
	move('A', Aspeed, direction);
	DEBUG_PRINT("Banking left");
}

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

	DEBUG_PRINT("Turning");

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

	#ifdef DEBUG
	Serial.print("Turning: direction; degrees; speed; distToTurn		");
	Serial.print(direction);
	Serial.print("; ");
	Serial.print(degrees);
	Serial.print("; ");
	Serial.print(speed);
	Serial.print("; ");
	Serial.print(distToTurn);
	Serial.print("	DistTurned: ");
	Serial.println(distTurned);
	#endif // DEBUG
}


// Calculate wheel encoder values and modify <<encValues[]>>
// to keep rotations, distance and velocity based on encoder.
void calculateEncValues(RotEncoder enc,
						volatile unsigned int ticks,
						unsigned long &oldMillis,
						float oldDistance,
						float encValues[]) {

	//::Encoder variables with RotEncoder library::
	// Number of rotations in floating point
	volatile float rotations = enc.rotations(ticks);
	// Distance, in relation to wheel size.
	float distance = enc.distance(ticks);
	// Velocity, in relation to distance and interval that is
	// specified here.
	unsigned long interval = millis() - oldMillis;
	float velocity = -1;
	if (interval > 800) {
		float velocity = enc.velocity(ticks, interval, oldDistance);
		oldMillis = millis();
	}

	#ifdef DEBUG
	Serial.print("Rotations: ");
	Serial.print(rotations);
	Serial.print(" ; Distance: ");
	Serial.print(distance);
	Serial.print(" m");
	Serial.print(" ; Speed: ");
	Serial.print(velocity);
	Serial.println(" m/s");
	delay(10);
	#endif // DEBUG
	
	encValues[0] = rotations;
	encValues[1] = distance;
	encValues[2] = velocity;

}

// Tones for piezo speaker
void makeTone(int notes[], int noteDurations[], int length) {
	// Iterate over tones
	for (int thisNote = 0; thisNote < length; thisNote++) {
		int noteDuration = 300 / noteDurations[thisNote];
		tone(SPEAKER_PIN, notes[thisNote], noteDuration);

		// Pause to let the tone play, now with 30% extra pause
		int pauseAfterTone = noteDuration * 1.30;
		delay(pauseAfterTone);
		noTone(SPEAKER_PIN);     // Stop playing tone
	}
	irrecv.enableIRIn();
	return;
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
