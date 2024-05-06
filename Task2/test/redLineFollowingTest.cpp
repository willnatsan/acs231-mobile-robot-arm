#include <Arduino.h>
#include <ArduinoSTL.h>

/* Right Motor Pins */
unsigned char pwmValueL = 125;
const int pinAI1R = 33;
const int pinAI2R = 31;
const int pinPWMAR = 4; // Pin allocation for the PWMA pin
boolean AI1R = 0;
boolean AI2R = 0;

/* Left Motor Pins */
unsigned char pwmValueR = 125;
const int pinBI1L = 41;
const int pinBI2L = 43;
const int pinPWMBL = 3; // Pin allocation for the PWMB pin
boolean BI1L = 0;
boolean BI2L = 0;

/* Other Motor Driver Pins */
const int pinStandBy = 5; // Pin allocation for the standby pin
boolean standBy = 0;      // standBy pin Value
boolean rotDirect = 0;    // Rotation direction variable

/* Line Following Sensor Pins and Sensor Reading Variables */
int LeftSensor = 0;
int CentreSensor = 1;
int RightSensor = 2;

int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

/* Ultrasonic Pins and Variables */
const int trigPin = 13;
const int echoPin = 12;

long duration;
int distance;

void setup() {
  Serial.begin(9600);

  /* Motor Pins */
  pinMode(pinAI1R, OUTPUT);
  pinMode(pinAI2R, OUTPUT);
  pinMode(pinPWMAR, OUTPUT);

  pinMode(pinBI1L, OUTPUT);
  pinMode(pinBI2L, OUTPUT);
  pinMode(pinPWMBL, OUTPUT);

  pinMode(pinStandBy, OUTPUT);
  standBy = true;
  digitalWrite(pinStandBy, standBy);

  /* Line Following Sensor Pins */
  pinMode(LeftSensor, INPUT);
  pinMode(CentreSensor, INPUT);
  pinMode(RightSensor, INPUT);

  /* Ultrasonic Pins */
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
}

void loop() {}

void followRedLine() {
  Serial.println("Following Red Line");
  AI1R = true;
  AI2R = false;
  BI1L = false;
  BI2L = true;
  LSreading = analogRead(LeftSensor);
  CSreading = analogRead(CentreSensor);
  RSreading = analogRead(RightSensor);

  Serial.print("Left Sensor Reading = ");
  Serial.print(LSreading);
  Serial.print("\t");

  Serial.print("Centre Sensor Reading = ");
  Serial.print(CSreading);
  Serial.print("\t");

  Serial.print("Right Sensor Reading = ");
  Serial.println(RSreading);

  do {
    AI1R = true;
    AI2R = false;
    BI1L = false;
    BI2L = true;
    LSreading = analogRead(LeftSensor);
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);

    Serial.print("Left Sensor Reading = ");
    Serial.print(LSreading);
    Serial.print("\t");
    Serial.print("Centre Sensor Reading = ");
    Serial.print(CSreading);
    Serial.print("\t");
    Serial.print("Right Sensor Reading = ");
    Serial.println(RSreading);

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;

    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    if ((CSreading > 600) && (LSreading > 600) && (RSreading > 600)) {
      analogWrite(pinPWMAR, 80);
      analogWrite(pinPWMBL, 80);
      // Serial.println("Going straight");
    } else if ((RSreading < 400) && (LSreading > 600)) {
      analogWrite(pinPWMAR, 90);
      analogWrite(pinPWMBL, 70);
      // Serial.println("Too much to right! Turning left");

    } else if ((LSreading < 400) && (RSreading > 600)) {
      analogWrite(pinPWMAR, 70);
      analogWrite(pinPWMBL, 90);
      // Serial.println("Too much to left! Turning right");
    } else if ((LSreading < 400) && (CSreading < 400) & (RSreading < 400)) {
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 50);
      // Serial.println("Moving straight slow");
    }
  } while (distance > 10);

  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
}