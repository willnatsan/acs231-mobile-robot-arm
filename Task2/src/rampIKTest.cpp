#include <Arduino.h>
#include <ArduinoSTL.h>
#include <Ramp.h>
#include <Servo.h>

/* Servo Objects & Pins */
Servo shoulder;
Servo elbow;
Servo wrist;

const uint8_t pinShoulder = 9;
const uint8_t pinElbow = 10;
const uint8_t pinWrist = 11;

// Declare variables, for use later:
float s1Angle;
float s1AnglePrev;
int s1Width;
float s2Angle;
float s2AnglePrev;
int s2Width;
float s3Angle;
float s3AnglePrev;
float s3AngleMax;
int s3Width;

float coordinatesInitial[] = {20.0, 6.3, 9.5};  // {x, y, z}
float coordinatesFinal[] = {23.0, -12.7, -9.5}; // {x , y, z}

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

const float L1 = 11.3;
const float L2 = 8;
const float L3 = 20.2;

unsigned long timeStamp;

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);

void setup() {
  delay(2000);
  Serial.begin(9600);
  timeStamp = millis();
  Serial.println("I'm in setup");

  shoulder.attach(pinShoulder);
  elbow.attach(pinElbow);
  wrist.attach(pinWrist);

  shoulder.writeMicroseconds(1580);
  elbow.writeMicroseconds(1470);
  wrist.writeMicroseconds(1550);
  delay(1000);

  // Ramp Calibration
  xRamp.go(28.2, 500, LINEAR, ONCEFORWARD);
  yRamp.go(0.0, 500, LINEAR, ONCEFORWARD);
  zRamp.go(0.0, 500, LINEAR, ONCEFORWARD);

  while (xRamp.update() != 28.2 || yRamp.update() != 0.0 ||
         zRamp.update() != 0.0) {
    xRamp.update();
    yRamp.update();
    zRamp.update();
  }

  // Initial Position
  xTarget = coordinatesInitial[0];
  yTarget = coordinatesInitial[1];
  zTarget = coordinatesInitial[2];

  xRamp.go(xTarget, 2000, LINEAR, ONCEFORWARD);
  yRamp.go(yTarget, 2000, LINEAR, ONCEFORWARD);
  zRamp.go(zTarget, 2000, LINEAR, ONCEFORWARD);

  while (xCurr != xTarget || yCurr != yTarget || zCurr != zTarget) {
    xCurr = xRamp.update();
    yCurr = yRamp.update();
    zCurr = zRamp.update();

    s1Angle = atan2(zCurr, xCurr);
    xCurrNew = sqrt(sq(zCurr) + sq(xCurr));
    s3Angle =
        -acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    s2Angle = atan2(yCurr, xCurrNew) -
              atan2(L3 * sin(s3Angle), L2 + L3 * cos(s3Angle));
    s1Width = moveServo(shoulder, 1580, s1Angle);
    s2Width = moveServo(elbow, 1470, s2Angle);
    s3Width = moveServo(wrist, 1550, s3Angle);

    timeStamp = millis();
  }

  // Horizontal Line Movement

  xTarget = coordinatesFinal[0];

  xRamp.go(xTarget, 3000, LINEAR, ONCEFORWARD);

  while (xCurr != xTarget) {
    xCurr = xRamp.update();

    s1Angle = atan2(zCurr, xCurr);
    xCurrNew = sqrt(sq(zCurr) + sq(xCurr));
    s3Angle =
        -acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    s2Angle = atan2(yCurr, xCurrNew) -
              atan2(L3 * sin(s3Angle), L2 + L3 * cos(s3Angle));
    s1Width = moveServo(shoulder, 1580, s1Angle);
    s2Width = moveServo(elbow, 1470, s2Angle);
    s3Width = moveServo(wrist, 1550, s3Angle);
  }

  delay(2000);

  // Diagonal Line Movement

  xTarget = coordinatesFinal[0];
  yTarget = coordinatesFinal[1];
  zTarget = coordinatesFinal[2];

  xRamp.go(xTarget, 6000, LINEAR, ONCEFORWARD);
  yRamp.go(yTarget, 6000, LINEAR, ONCEFORWARD);
  zRamp.go(zTarget, 6000, LINEAR, ONCEFORWARD);

  while (yCurr != yTarget || zCurr != zTarget) {
    xCurr = xRamp.update();
    yCurr = yRamp.update();
    zCurr = zRamp.update();

    s1Angle = atan2(zCurr, xCurr);
    xCurrNew = sqrt(sq(zCurr) + sq(xCurr));
    s3Angle =
        -acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    s2Angle = atan2(yCurr, xCurrNew) -
              atan2(L3 * sin(s3Angle), L2 + L3 * cos(s3Angle));
    s1Width = moveServo(shoulder, 1580, s1Angle);
    s2Width = moveServo(elbow, 1470, s2Angle);
    s3Width = moveServo(wrist, 1550, s3Angle);
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
