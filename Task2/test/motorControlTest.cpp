#include <Arduino.h>
#include <ArduinoSTL.h>
#include <PID_v1.h>

/* Right Motor Pins */
unsigned char pwmValueL = 125;
const uint8_t pinAI1R = 33;
const uint8_t pinAI2R = 31;
const uint8_t pinPWMAR = 4;

boolean AI1R = false;
boolean AI2R = false;
boolean directionR = true; // true for forward, false for backward
int pulseCountR = 0;

/* Left Motor Pins */
unsigned char pwmValueR = 125;
const uint8_t pinBI1L = 41;
const uint8_t pinBI2L = 43;
const uint8_t pinPWMBL = 3;

boolean BI1L = false;
boolean BI2L = false;
boolean directionL = true; // true for forward, false for backward
int pulseCountL = 0;

/* Other Motor Driver Pins & Values */
const uint8_t pinStandBy = 5;

boolean standBy = true;

/* Right Wheel Encoder Pins */
const int pinEncoderRightA = 18;
const int pinEncorderRightB = 19;
byte encoderRightALast;

/* Left Wheel Encoder Pins */
const int pinEncoderLeftA = 20;
const int pinEncoderLeftB = 21;
byte encoderLeftALast;

/* Encoder Variables */
const int encoderK = 12;
volatile long encCountRight;
volatile float encRevRight;
volatile long encCountLeft;
volatile float encRevLeft;

/* Functions Declarations */
void spin();
void straight();
void readLeftMotorSpeed();
void readRightMotorSpeed();

void setup() {

  /* Motor Pins */
  pinMode(pinAI1R, OUTPUT);
  pinMode(pinAI2R, OUTPUT);
  pinMode(pinPWMAR, OUTPUT);

  pinMode(pinBI1L, OUTPUT);
  pinMode(pinBI2L, OUTPUT);
  pinMode(pinPWMBL, OUTPUT);

  pinMode(pinStandBy, OUTPUT);
  digitalWrite(pinStandBy, standBy);

  /* Encoder Pins */
  pinMode(pinEncorderRightB, INPUT);
  pinMode(pinEncoderLeftB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderRightA), readLeftMotorSpeed,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderLeftA), readRightMotorSpeed,
                  CHANGE);

  Serial.begin(9600); // Starts the serial communication

  delay(2000);
}
void loop() {
  Serial.print("Left Pulse: ");
  Serial.print(pulseCountL);
  Serial.print("\tRight Pulse: ");
  Serial.println(pulseCountR);
  delay(100);
}

/* Spin 180 */
void spin() {
  AI1R = false;
  AI2R = true;
  BI1L = false;
  BI2L = true;
  int x = 0;
  int y = 0;
  while (encRevRight < 205) {
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    analogWrite(pinPWMAR, 100);
    analogWrite(pinPWMBL, 100);
  }
  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
}

/* Go Straight */
void straight() {
  AI1R = false;
  AI2R = true;
  BI1L = true;
  BI2L = false;

  while (true) {
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    analogWrite(pinPWMAR, 100);
    analogWrite(pinPWMBL, 100);
  }

  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
}

/*** Encoders Functions ***/
void readLeftMotorSpeed() {
  byte leftStateA = digitalRead(pinEncoderLeftA);
  if ((encoderLeftALast == LOW) && (leftStateA == HIGH)) {
    byte leftStateB = digitalRead(pinEncoderLeftB);
    if (leftStateB == LOW && directionL) {
      directionL = false;
    } else if (leftStateB == HIGH && !directionL) {
      directionL = true;
    }
  }
  encoderLeftALast = leftStateA;
  if (!directionL) {
    pulseCountL++;
  } else {
    pulseCountL--;
  }
}

void readRightMotorSpeed() {
  byte rightStateA = digitalRead(pinEncoderRightA);
  if ((encoderRightALast == LOW) && (rightStateA == HIGH)) {
    byte rightStateB = digitalRead(pinEncorderRightB);
    if (rightStateB == LOW && directionR) {
      directionR = false;
    } else if (rightStateB == HIGH && !directionR) {
      directionR = true;
    }
  }
  encoderRightALast = rightStateA;
  if (!directionR) {
    pulseCountR++;
  } else {
    pulseCountR--;
  }
}