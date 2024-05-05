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
unsigned long t0;

/* Ultrasonic Pins and Variables */
const int pinTrig = 13;
const int pinEcho = 12;

long duration;
int distance;

/* Line Following Sensor Pins and Sensor Reading Variables */
int pinLeftSensor = 0;
int pinCentreSensor = 1;
int pinRightSensor = 2;

int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

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

  t0 = millis();

  /* Line Follower Pins */
  pinMode(pinLeftSensor, INPUT);
  pinMode(pinCentreSensor, INPUT);
  pinMode(pinRightSensor, INPUT);

  /* Ultrasonic Pins */
  pinMode(pinTrig, OUTPUT); // Sets the trigPin as an Output
  pinMode(pinEcho, INPUT);  // Sets the echoPin as an Input

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
  int x = 0;
  int y = 0;

  do {
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);

    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(pinEcho, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    // Serial.print("Distance: ");
    // Serial.println(distance);x
    if (100 + x <= 120) {
      analogWrite(pinPWMAR, 100 + x);
    }
    if (100 + y <= 120) {
      analogWrite(pinPWMBL, 100 + y);
    }

    if (encRevRight < encRevLeft) {
      x += 3;
    }
    if (encRevLeft < encRevRight) {
      y += 3;
    }
  } while (distance > 15);
  Serial.print(distance);

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