#include <Arduino.h>
#include <ArduinoSTL.h>
#include <Ramp.h>
#include <Servo.h>

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

volatile long enc_count_right = 0;
volatile float enc_rev_right = 0;
volatile long enc_count_left = 0;
volatile float enc_rev_left = 0;
unsigned long t0;
float prevSpeed = 90;

/* Ultrasonic Pins and Variables */
const int trigPin = 13;
const int echoPin = 12;

long duration;
int distance;

/* Servo Objects & Pins */
Servo shoulder;
Servo elbow;
Servo wrist;

const uint8_t pinShoulder = A5;
const uint8_t pinElbow = A6;
const uint8_t pinWrist = A7;

// Declare variables, for use later:
float s1Angle;
int s1Width;
float s2Angle;
int s2Width;
float s3Angle;
int s3Width;

// WORKING: DO NOT CHANGE
// float coordinatesInitial[] = {19, 9.5, 9.5};   // {x, y, z}
// float coordinatesFinal[] = {20.0, -9.5, -9.5}; // {x , y, z}

float coordinatesInitial[] = {19.0, 12.8, 6.5}; // {x, y, z}
float coordinatesFinal[] = {20.5, -4.0, -10.5}; // {x , y, z}

float xCurr;
float xCurrNew;
float yCurr;
float zCurr;

float xTarget;
float yTarget;
float zTarget;

rampFloat xRamp;
rampFloat yRamp;
rampFloat zRamp;

const int SETUP_TIME = 2500;
const int HORIZONTAL_MOVE_TIME = 2500;
const int DIAGONAL_MOVE_TIME = 2500;

const float L1 = 11.3;
const float L2 = 8;
const float L3 = 17.5;

unsigned long timeStamp;
bool pauseFlag = true;

/* Line Following Sensor Pins and Sensor Reading Variables */
int LeftSensor = 0;
int CentreSensor = 1;
int RightSensor = 2;

int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

// Variables for PID Control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

/* Functions Declarations */
void spin(float rot);
void straight();
void followBlackLine();
void armInitialise();
void drawDiagonal();
void followRedLine();
void channelA();
void channelB();
void channelC();
void channelD();
void moveMotor(float u);
float pidController(int target, float kp, float kd, float ki);
int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);

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
  // pinMode(PINB, INPUT);
  pinMode(PINC, INPUT);
  // pinMode(PIND, INPUT);
  attachInterrupt(digitalPinToInterrupt(PINA), channelA, RISING);
  // attachInterrupt(digitalPinToInterrupt(PINB), channelB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINC), channelC, RISING);
  // attachInterrupt(digitalPinToInterrupt(PIND), channelD, CHANGE);
  t0 = millis();

  /* Line Follower Pins */
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);

  /* Ultrasonic Pins */
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication

  shoulder.attach(pinShoulder);
  elbow.attach(pinElbow);
  wrist.attach(pinWrist);

  shoulder.writeMicroseconds(1532);
  elbow.writeMicroseconds(2350);
  wrist.writeMicroseconds(1487);
  delay(1000);

  delay(2000);

  spin(223 / 2);
  straight();
  followBlackLine();
  drawDiagonal();

  Serial.println("Reaches HERE");

  shoulder.writeMicroseconds(615);
  delay(1000);

  elbow.writeMicroseconds(2350);
  wrist.writeMicroseconds(1487);
  delay(1000);

  spin(209 / 4);

  delay(2000);
  straight();
  spin(205 / 4);

  shoulder.writeMicroseconds(1532);
  delay(1000);
  elbow.writeMicroseconds(1450);
  wrist.writeMicroseconds(1487);

  delay(2000);
  straight();

  while (1) {
  }
}
void loop() {}

/********************************************** Functions
 * **********************************************/

/* Spin 180 */
void spin(float rot) {
  enc_count_right = 0;
  enc_count_left = 0;
  enc_rev_right = 0;
  enc_rev_left = 0;
  delay(1000);
  AI1R = true;
  AI2R = false;
  BI1L = true;
  BI2L = false;
  int x = 0;
  int y = 0;
  while (enc_rev_right < rot) {
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    analogWrite(pinPWMAR, 120);
    analogWrite(pinPWMBL, 120);

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
  enc_count_left = 0;
  enc_count_right = 0;
  Serial.print(enc_count_left);
  Serial.print("\t");
  Serial.println(enc_count_right);
  Serial.println("Straight");
  delay(1500);
}

/* Go Straight */
void straight() {
  enc_count_right = 0;
  enc_count_left = 0;
  enc_rev_right = 0;
  enc_rev_left = 0;
  delay(1000);
  Serial.print(enc_count_left);
  Serial.print("\t");
  Serial.println(enc_count_right);
  Serial.println("Straight");
  AI1R = true;
  AI2R = false;
  BI1L = false;
  BI2L = true;
  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);
  analogWrite(pinPWMAR, 90);
  analogWrite(pinPWMBL, 70);
  delay(200);
  int x = 0;
  int y = 0;

  do {
    int target = enc_count_right;
    analogWrite(pinPWMAR, 90);

    // PID controller gains and computation
    float kp = 2.0;
    float kd = 0.0;
    float ki = 0.03;
    float u = pidController(target, kp, kd, ki);
    // Control motor 2 based on PID
    moveMotor(u);

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
    Serial.print("Distance:");
    Serial.println(distance);
  } while (distance > 15);
  // distance > 15
  Serial.print(distance);

  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
}

/* Follow Black Line */
void followBlackLine() {
  Serial.println("Following");
  do {
    AI1R = true;
    AI2R = false;
    BI1L = true;
    BI2L = false;
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    analogWrite(pinPWMAR, 60);
    analogWrite(pinPWMBL, 60);
    LSreading = analogRead(LeftSensor);
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);

  } while (CSreading < 450);

  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  shoulder.writeMicroseconds(1532);
  elbow.writeMicroseconds(1450);
  wrist.writeMicroseconds(1487);
  delay(3000);

  do {
    AI1R = true;
    AI2R = false;
    BI1L = false;
    BI2L = true;
    LSreading = analogRead(LeftSensor);
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);
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

    if (distance < 19.2) {
      shoulder.writeMicroseconds(1532);
      elbow.writeMicroseconds(2350);
      wrist.writeMicroseconds(1487);
    }

    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    // if ((CSreading > 430) || (LSreading > 550) & (RSreading > 550) &&
    // distance < 18.2) {
    if (LSreading > 430) {
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 50);
      // straight(11.7);
      // Serial.println("Going straight");
    } else if ((RSreading < 400) && (CSreading < 430) && (LSreading > 550)) {
      analogWrite(pinPWMAR, 70);
      analogWrite(pinPWMBL, 50);
      // Serial.println("Too much to right! Turning left");

    } else if ((LSreading < 400) && (CSreading < 430) && (RSreading > 550)) {
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 70);
      // Serial.println("Too much to left! Turning right");
    } else if ((LSreading > 400) && (CSreading > 400) && (RSreading < 400)) {
      analogWrite(pinPWMAR, 60);
      analogWrite(pinPWMBL, 50);
    } else if ((LSreading < 400) && (CSreading > 400) && (RSreading > 400)) {
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 60);
      // Serial.println("Moving straight slow");
    }
  } while (distance > 11.2);
  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
}

void drawDiagonal() {
  shoulder.writeMicroseconds(1532);
  elbow.writeMicroseconds(2350);
  wrist.writeMicroseconds(1487);
  delay(1000);

  // Ramp Calibration
  xRamp.go(0.0, 500, LINEAR, ONCEFORWARD);
  yRamp.go(L2 + L3, 500, LINEAR, ONCEFORWARD);
  zRamp.go(0.0, 500, LINEAR, ONCEFORWARD);

  while (xRamp.update() != 0.0 || yRamp.update() != L2 + L3 ||
         zRamp.update() != 0.0) {
    xRamp.update();
    yRamp.update();
    zRamp.update();
  }

  // Initial Position
  xTarget = coordinatesInitial[0];
  yTarget = coordinatesInitial[1];
  zTarget = coordinatesInitial[2];

  xRamp.go(xTarget, SETUP_TIME, LINEAR, ONCEFORWARD);
  yRamp.go(yTarget, SETUP_TIME, LINEAR, ONCEFORWARD);
  zRamp.go(zTarget, SETUP_TIME, LINEAR, ONCEFORWARD);

  Serial.println("\nInitial Position Movement");

  while (xCurr != xTarget || yCurr != yTarget || zCurr != zTarget) {
    xCurr = xRamp.update();
    yCurr = yRamp.update();
    zCurr = zRamp.update();

    s1Angle = atan2(zCurr, xCurr);
    xCurrNew = sqrt(sq(zCurr) + sq(xCurr));
    // s3Angle =
    //     acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    // s2Angle = atan2(yCurr, xCurrNew) -
    //           atan2(L3 * sin(-s3Angle), L2 + L3 * cos(-s3Angle));
    float n = sqrt(sq(xCurrNew) + sq(yCurr));
    s3Angle = PI - acos((sq(L2) + sq(L3) - sq(n)) / (2 * L2 * L3));
    s2Angle =
        atan2(yCurr, xCurrNew) + acos((sq(L2) + sq(n) - sq(L3)) / (2 * L2 * n));
    s1Width = moveServo(shoulder, 1532, s1Angle);
    s2Width = moveServo(elbow, 1450, s2Angle);
    s3Width = moveServo(wrist, 1487, s3Angle);
  }

  delay(2000);

  // Horizontal Line Movement

  xTarget = coordinatesFinal[0];

  xRamp.go(xTarget, HORIZONTAL_MOVE_TIME, LINEAR, ONCEFORWARD);

  Serial.println("\nHorizontal Line Movement");

  while (xCurr != xTarget) {

    xCurr = xRamp.update();

    s1Angle = atan2(zCurr, xCurr);
    xCurrNew = sqrt(sq(zCurr) + sq(xCurr));
    // s3Angle =
    //     acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    // s2Angle = atan2(yCurr, xCurrNew) -
    //           atan2(L3 * sin(-s3Angle), L2 + L3 * cos(-s3Angle));
    float n = sqrt(sq(xCurrNew) + sq(yCurr));
    s3Angle = PI - acos((sq(L2) + sq(L3) - sq(n)) / (2 * L2 * L3));
    s2Angle =
        atan2(yCurr, xCurrNew) + acos((sq(L2) + sq(n) - sq(L3)) / (2 * L2 * n));
    s1Width = moveServo(shoulder, 1532, s1Angle);
    s2Width = moveServo(elbow, 1450, s2Angle);
    s3Width = moveServo(wrist, 1487, s3Angle);
  }

  delay(2000);
  Serial.println("\n\n\n\n");

  // Diagonal Line Movement

  xTarget = coordinatesFinal[0];
  yTarget = coordinatesFinal[1];
  zTarget = coordinatesFinal[2];

  xRamp.go(xTarget, DIAGONAL_MOVE_TIME, LINEAR, ONCEFORWARD);
  yRamp.go(yTarget, DIAGONAL_MOVE_TIME, LINEAR, ONCEFORWARD);
  zRamp.go(zTarget, DIAGONAL_MOVE_TIME, LINEAR, ONCEFORWARD);

  Serial.println("\nDiagonal Line Movement");

  timeStamp = millis();

  while (yCurr != yTarget || zCurr != zTarget) {

    xCurr = xRamp.update();
    yCurr = yRamp.update();
    zCurr = zRamp.update();

    s1Angle = atan2(zCurr, xCurr);
    xCurrNew = sqrt(sq(zCurr) + sq(xCurr));
    // s3Angle =
    //     acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    // s2Angle = atan2(yCurr, xCurrNew) -
    //           atan2(L3 * sin(-s3Angle), L2 + L3 * cos(-s3Angle));
    float n = sqrt(sq(xCurrNew) + sq(yCurr));
    s3Angle = PI - acos((sq(L2) + sq(L3) - sq(n)) / (2 * L2 * L3));
    s2Angle =
        atan2(yCurr, xCurrNew) + acos((sq(L2) + sq(n) - sq(L3)) / (2 * L2 * n));
    s1Width = moveServo(shoulder, 1532, s1Angle);
    s2Width = moveServo(elbow, 1450, s2Angle);
    s3Width = moveServo(wrist, 1487, s3Angle);

    Serial.print("yCurr: ");
    Serial.print(yCurr);
    Serial.print("\tzCurr: ");
    Serial.println(zCurr);
  }
  delay(2000);
}

void followRedLine() {}

/*** Encoders Functions ***/
void channelA() {
  // if (digitalRead(PINA) == HIGH) {
  //   if (digitalRead(PINB == LOW))
  //     enc_count_right++;
  //   else
  //     enc_count_right--;
  // } else {
  //   if (digitalRead(PINB) == HIGH)
  //     enc_count_right++;
  //   else
  //     enc_count_right--;
  // }
  enc_count_right++;
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
  // if (digitalRead(PINC) == HIGH) {
  //   if (digitalRead(PIND == LOW))
  //     enc_count_left++;
  //   else
  //     enc_count_left--;
  // } else {
  //   if (digitalRead(PIND) == HIGH)
  //     enc_count_left++;
  //   else
  //     enc_count_left--;
  // }
  enc_count_left++;
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

void moveMotor(float u) {
  // Maximum motor speed
  float speed;
  if (u < -90 || u > 90) {
    speed = fabs(u);
  } else {
    speed = prevSpeed;
  }
  if (speed > 103) {
    speed = 103;
  }
  BI1L = false;
  BI2L = true;
  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);
  // Stop the motor during overshoot
  //  if (enc_count_left > enc_count_right){
  //      //speed = 0;
  //      // digitalWrite(LEDpin,HIGH);
  //      // delay(200);
  //      // digitalWrite(LEDpin,LOW);
  //      // delay(200);
  //      speed = speed -20;
  //  }
  // Control the motor
  prevSpeed = speed;
  analogWrite(pinPWMBL, speed);
}
float pidController(int target, float kp, float kd, float ki) {
  // Measure time elapsed since the last iteration
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  // Compute the error, derivative, and integral
  int e = target - enc_count_left;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  // Compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  // Update variables for the next iteration
  previousTime = currentTime;
  ePrevious = e;
  return u;
}

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle) {
  float minPulseWidth = float(refPulseWidth) - 1000.0;
  int cmdSignal = (servoRadAngle + (PI / 2)) * (2000.0 / PI) + minPulseWidth;
  servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}
