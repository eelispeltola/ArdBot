/* 
 * ArdBot v1.0.
 * Arduino Nano, ATmega328.
 * Two continuos rotation servos, piezo speaker.
 * Sensors: IR diode (Sony remote controlled), two leaf switches.
 * Started 26.08.2015.
 * Last modified 12.09.2015.
 * epe
 */

#include <IRremote.h>
#include <Servo.h>
#include "pitches.h"

// Define servos
Servo servoLeft;
Servo servoRight;

// Define IRremote objects
int RECV_PIN = 5;
IRrecv irrecv(RECV_PIN);
decode_results results;

// Define variables for bumpers
volatile int bLeft = LOW;
volatile int bRight = LOW;

boolean started = false;

void setup() {
  // Set pin modes for switches
  pinMode(2, INPUT);     // Redundant code, all pins def inputs
  pinMode(3, INPUT);     // See above
  pinMode(4, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);     // Ground

  // Set pin modes for IR receiver and speaker
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, LOW);     // IR ground
  digitalWrite(7, HIGH);     // IR Power
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);     // Speaker ground

  servoLeft.attach(9);     // Set left servo to be driven on D8
  servoRight.attach(8);
  irrecv.enableIRIn();     // Start IR receiver

  Serial.begin(9600);

  // Set interrupts
  attachInterrupt(1, hitRight, FALLING);
  attachInterrupt(0, hitLeft, FALLING);

  started = true;
}

void loop() {
  if (bLeft == HIGH) {          // If left bumper hit
    Serial.println("bLeft");
    int notes[] = {NOTE_FS3, NOTE_F3, NOTE_D3};
    int noteDurations[] = {8,4,8};
    reverse();
    makeTone(notes, noteDurations, sizeof(notes)/sizeof(int));
    delay(500);
    spinRight();
    delay(1500);
    forward();
    bLeft = LOW;
  }
  
  if (bRight == HIGH) {          // If right bumper hit
    Serial.println("bRight");
    int notes[] = {NOTE_F4, NOTE_D4, NOTE_B3};
    int noteDurations[] = {4,16,8};
    reverse();
    makeTone(notes, noteDurations, sizeof(notes)/sizeof(int));
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
}

void makeTone(int notes[], int noteDurations[], int length) {
  // Iterate over tones
  for (int thisNote = 0; thisNote < length; thisNote++) {
    int noteDuration = 1000/noteDurations[thisNote];
    tone(12, notes[thisNote], noteDuration);
    
    // Pause to let the tone play, now with 30% extra pause
    int pauseAfterTone = noteDuration * 1.30;
    delay(pauseAfterTone);
    noTone(12);     // Stop playing tone
  }
  irrecv.enableIRIn();
  return;
}

// Servo routines for motions
void forward() {
  servoLeft.write(180);
  servoRight.write(0);
}
void reverse() {
  servoLeft.write(0);
  servoRight.write(180);
}
void stopRobot() {
  servoLeft.write(90);
  servoRight.write(90);
}
void spinLeft() {
  servoLeft.write(0);
  servoRight.write(0);
}
void spinRight() {
  servoLeft.write(180);
  servoRight.write(180);
}
void turnLeftFwd() {
  servoLeft.write(90);
  servoRight.write(0);
}
void turnRightFwd() {
  servoLeft.write(180);
  servoRight.write(90);
}
void turnLeftRev() {
  servoLeft.write(90);
  servoRight.write(180);
}
void turnRightRev() {
  servoLeft.write(90);
  servoRight.write(180);
}

// Interrupt handlers
void hitLeft() {
  if(started)
    bLeft = HIGH;
}
void hitRight() {
  if(started)
    bRight = HIGH;
}
