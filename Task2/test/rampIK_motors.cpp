#include <Arduino.h>
#include <ArduinoSTL.h>
#include <Ramp.h>
#include <Servo.h>

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

float coordinatesInitial[] = {19, 12.5, 5.5};   // {x, y, z}
float coordinatesFinal[] = {20.0, -5.5, -12.5}; // {x , y, z}

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

const int SETUP_TIME = 2000;
const int HORIZONTAL_MOVE_TIME = 1000;
const int DIAGONAL_MOVE_TIME = 1300;

const float L1 = 11.3;
const float L2 = 8;
const float L3 = 17.5;

unsigned long timeStamp;
bool pauseFlag = true;

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);

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
float prevSpeed = 90;

// Variables for PID Control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

/* Ultrasonic Pins and Variables */
const int trigPin = 13;
const int echoPin = 12;

long duration;
int distance;

void straight();
void moveMotor(float u);
float pidController(int target, float kp, float kd, float ki);
void channelA();
void channelC();

void setup() {
  delay(2000);
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

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  Serial.begin(9600);
  Serial.println("I'm in setup");

  t0 = millis();

  straight();

  /* Ultrasonic Pins */

  shoulder.attach(pinShoulder);
  elbow.attach(pinElbow);
  wrist.attach(pinWrist);

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

    // Serial.print("s1Angle: ");
    // Serial.print(s1Angle);
    // Serial.print("\ts2Angle: ");
    // Serial.print(s2Angle);
    // Serial.print("\ts3Angle: ");
    // Serial.println(s3Angle);

    Serial.print("xCurrNew: ");
    Serial.print(L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle));
    Serial.print("\txCurr: ");
    Serial.print((L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle)) *
                 cos(s1Angle));
    Serial.print("\tyCurr: ");
    Serial.print(L2 * sin(s2Angle) + L3 * sin(s2Angle - s3Angle));
    Serial.print("\tzCurr: ");
    Serial.println((L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle)) *
                   sin(s1Angle));
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

    // Serial.print("s1Angle: ");
    // Serial.print(s1Angle);
    // Serial.print("\ts2Angle: ");
    // Serial.print(s2Angle);
    // Serial.print("\ts3Angle: ");
    // Serial.println(s3Angle);

    Serial.print("xCurrNew: ");
    Serial.print(L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle));
    Serial.print("\txCurr: ");
    Serial.print((L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle)) *
                 cos(s1Angle));
    Serial.print("\tyCurr: ");
    Serial.print(L2 * sin(s2Angle) + L3 * sin(s2Angle - s3Angle));
    Serial.print("\tzCurr: ");
    Serial.println((L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle)) *
                   sin(s1Angle));
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

    // if (millis() - timeStamp >= DIAGONAL_MOVE_TIME / 4 && pauseFlag) {
    //   // MOVE ROBOT BACK HERE
    //   Serial.println("Paused");
    //   delay(5000);

    //   xRamp.go(xTarget, DIAGONAL_MOVE_TIME, LINEAR, ONCEFORWARD);
    //   yRamp.go(yTarget, DIAGONAL_MOVE_TIME, LINEAR, ONCEFORWARD);
    //   zRamp.go(zTarget, DIAGONAL_MOVE_TIME, LINEAR, ONCEFORWARD);

    //   pauseFlag = false;
    // }

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

    // Serial.print("s1Angle: ");
    // Serial.print(s1Angle);
    // Serial.print("\ts2Angle: ");
    // Serial.print(s2Angle);
    // Serial.print("\ts3Angle: ");
    // Serial.println(s3Angle);

    Serial.print("xCurrNew: ");
    Serial.print(L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle));
    Serial.print("\txCurr: ");
    Serial.print((L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle)) *
                 cos(s1Angle));
    Serial.print("\tyCurr: ");
    Serial.print(L2 * sin(s2Angle) + L3 * sin(s2Angle - s3Angle));
    Serial.print("\tzCurr: ");
    Serial.println((L2 * cos(s2Angle) + L3 * cos(s2Angle - s3Angle)) *
                   sin(s1Angle));
  }

  while (1) {
  }
}

void loop() {}

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle) {
  float minPulseWidth = float(refPulseWidth) - 1000.0;
  int cmdSignal = (servoRadAngle + (PI / 2)) * (2000.0 / PI) + minPulseWidth;
  servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}

void straight() {
  enc_count_left = 0;
  enc_count_right = 0;
  delay(500);
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
  } while (distance > 11.2);
  // distance > 15
  Serial.print(distance);

  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
}

void channelA() {
  enc_count_right++;
  enc_rev_right = (float)enc_count_right / ENC_K;
}

void channelC() {
  enc_count_left++;
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
  if (speed > 110) {
    speed = 110;
  }
  BI1L = false;
  BI2L = true;
  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);
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