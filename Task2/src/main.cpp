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

/* Encoder Pins and Variables */
#define PINA 18
#define PINB 19
#define PINC 20
#define PIND 21
#define ENC_K 12

volatile long enc_count_right;
volatile float enc_rev_right;
volatile long enc_count_left;
volatile float enc_rev_left;
unsigned long t0;

/* Ultrasonic Pins and Variables */
const int trigPin = 13;
const int echoPin = 12;

long duration;
int distance;

/* Line Following Sensor Pins and Sensor Reading Variables */
int LeftSensor = 0;
int CentreSensor = 1;
int RightSensor = 2;

int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

/* Functions Declarations */
void spin();
void straight();
void followBlackLine();
void armInitialise();
void drawDiagonal();
void followRedLine();
void channelA();
void channelB();
void channelC();
void channelD();

void setup() {

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

  /* Encoder Pins */
  pinMode(PINA, INPUT);
  pinMode(PINB, INPUT);
  attachInterrupt(digitalPinToInterrupt(PINA), channelA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINB), channelB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINC), channelC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIND), channelD, CHANGE);
  t0 = millis();

  /* Line Follower Pins */
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);

  /* Ultrasonic Pins */
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication

  delay(2000);

  /* Spin 180 */
  spin();

  /* Go Straight */
  straight();

  /* Follow Black Line*/
  followBlackLine();

  while (1) {
  }
}
void loop() {}

/********************************************** Functions
 * **********************************************/

/* Spin 180 */
void spin() {
  AI1R = true;
  AI2R = false;
  BI1L = true;
  BI2L = false;
  int x = 0;
  int y = 0;
  while (enc_rev_right < 205) {
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    analogWrite(pinPWMAR, 100);
    //analogWrite(pinPWMBL, 100);

    // if (millis() - t0 > 20) {
    //   Serial.print("Encoder right count: ");
    //   Serial.print(enc_count_right);
    //   Serial.print("\tEncoder right revolution: ");
    //   Serial.print(enc_rev_right);

    //   Serial.print("\nEncoder left count: ");
    //   Serial.print(enc_count_left);
    //   Serial.print("\tEncoder left revolution: ");
    //   Serial.println(enc_rev_left);
    // if (enc_rev_right < enc_rev_left) {
    //   x += 3;
    // }
    // if (enc_rev_left < enc_rev_right) {
    //   y += 3;
    // }
    // }
  }
  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
}

/* Go Straight */
void straight() {
  Serial.println("Straight");
  AI1R = true;
  AI2R = false;
  BI1L = false;
  BI2L = true;
  int x = 0;
  int y = 0;

  do {
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);

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
    // Prints the distance on the Serial Monitor
    // Serial.print("Distance: ");
    // Serial.println(distance);x
    analogWrite(pinPWMAR,85);
    analogWrite(pinPWMBL,100);
    Serial.print("Distance: ");
    Serial.println(distance);
  //   if (100 + x <= 120) {
  //     analogWrite(pinPWMAR, 100 + x);
  //   }
  //   if (100 + y <= 120) {
  //     analogWrite(pinPWMBL, 100 + y);
  //   }

  //   if (enc_rev_right < enc_rev_left) {
  //     x += 3;
  //   }
  //   if (enc_rev_left < enc_rev_right) {
  //     y += 3;
  //   }
  } while (distance > 15);
  Serial.print(distance);

  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
  
}

/* Follow Black Line */
void followBlackLine() {
  Serial.println("Following");
  int x = 0;
  int y = 0;
  do {
    AI1R = true;
    AI2R = false;
    BI1L = true;
    BI2L = false;
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    LSreading = analogRead(LeftSensor);
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);
    analogWrite(pinPWMAR, 55 + x);
    analogWrite(pinPWMBL, 55 + y);
    if (enc_rev_right < enc_rev_left) {
      x += 3;
    }
    if (enc_rev_left < enc_rev_right) {
      y += 3;
    }
  } while (CSreading < 600);

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
    // Prints the distance on the Serial Monitor
    // Serial.print("Distance: ");
    // Serial.println(distance);

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

void armInitialise() {}
void drawDiagonal() {}
void followRedLine() {}

/*** Encoders Functions ***/
void channelA() {
  if (digitalRead(PINA) == HIGH) {
    if (digitalRead(PINB == LOW))
      enc_count_right++;
    else
      enc_count_right--;
  } else {
    if (digitalRead(PINB) == HIGH)
      enc_count_right++;
    else
      enc_count_right--;
  }
  enc_rev_right = (float)enc_count_right / ENC_K;
}

void channelB() {
  if (digitalRead(PINB) == HIGH) {
    if (digitalRead(PINA == LOW))
      enc_count_right++;
    else
      enc_count_right--;
  } else {
    if (digitalRead(PINA) == HIGH)
      enc_count_right++;
    else
      enc_count_right--;
  }
  enc_rev_right = (float)enc_count_right / ENC_K;
}

void channelC() {
  if (digitalRead(PINC) == HIGH) {
    if (digitalRead(PIND == LOW))
      enc_count_left++;
    else
      enc_count_left--;
  } else {
    if (digitalRead(PIND) == HIGH)
      enc_count_left++;
    else
      enc_count_left--;
  }
  enc_rev_left = (float)enc_count_left / ENC_K;
}

void channelD() {
  if (digitalRead(PIND) == HIGH) {
    if (digitalRead(PINC == LOW))
      enc_count_left++;
    else
      enc_count_left--;
  } else {
    if (digitalRead(PINC) == HIGH)
      enc_count_left++;
    else
      enc_count_left--;
  }
  enc_rev_left = (float)enc_count_left / ENC_K;
}
