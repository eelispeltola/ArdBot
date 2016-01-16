/* 
 * ArdBot v1.1.
 * Arduino Nano, ATmega328.
 * Two continuos rotation servos, piezo speaker.
 * Sensors: IR diode (Sony remote controlled), two IR
 * receivers and IR LEDs.
 * Started 16.01.2016.
 * Last modified 16.01.2016.
 * epe
 */

#include <RotEncoder.h>
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

// Define encoders
RotEncoder leftenc(68.5, 2);  // wheel diam (mm), encoder magnets
RotEncoder rightenc(68.5, 2);  // wheel diam (mm), encoder magnets

// Define variables for bumpers
volatile int bLeft = LOW;
volatile int bRight = LOW;

// Define variables for encoders
volatile unsigned int clicksLeft = 0;
volatile unsigned int clicksRight = 0;

// Define variables for debouncing
unsigned int debDelay = 20;
unsigned long lastDebounceLeft = 0;
unsigned long lastDebounceRight = 0;

// Define variables for velocity reading
unsigned long oldMillis = 0;
float oldDist = 0;
float vel = 0;

boolean started = false;

void setup() {
  // Set pin modes for encoders
  // TODO: Rewire grounds to breadboard GND
  digitalWrite(2, HIGH);    // Left enc
  digitalWrite(3, HIGH);    // Right enc

  // Set pin modes for microswitches
  const int leftSwitch = 10;
  const int RightSwitch = 4;
  digitalWrite(leftSwitch, HIGH);
  digitalWrite(RightSwitch, HIGH);
  
  // Set pin modes for IR receiver and speaker
  // TODO: Grounds to breadboard GND
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, LOW);     // IR ground
  digitalWrite(7, HIGH);     // IR Power
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);     // Speaker ground

  servoLeft.attach(9);     // Set left servo to be driven on D9
  servoRight.attach(8);
  irrecv.enableIRIn();     // Start IR receiver

  Serial.begin(9600);

  // Set interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(2), trigLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), trigRight, FALLING);
  
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

  // Number of rotations for both wheels
  volatile float leftrotations = leftenc.count(clickcounts);
  volatile float rightrotations = rightenc.count(clickcounts);

  // Distance as avg of both wheels
  float dist = (leftenc.distance(clickcounts) +
               righttenc.distance(clickcounts))/2;

  // Velocity as avg of both wheels
  unsigned long interval = millis() - oldMillis;
  if (interval > 2000) {
    vel = (leftenc.velocity(clickcounts, interval, oldDist) + 
          rightenc.velocity(clickcounts, interval, oldDist))/2;
    
    Serial.print("Rotations (left): ";
    Serial.print(leftrotations);
    Serial.print("Rotations (right): ";
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
void trigLeft() {
  if (started and 
      (millis() - lastDebounceLeft) > debDelay) {
    clicksLeft++;
    lastDebounceLeft = millis();
  }
}
void trigRight() {
  if (started and 
      (millis() - lastDebounceRight) > debDelay) {
    clicksRight++;
    lastDebounceRight = millis();
  }
}
