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
int s1Width;
float s2Angle;
int s2Width;
float s3Angle;
int s3Width;

float coordinatesInitial[] = {19, 8.5, 9.5};    // {x, y, z}
float coordinatesFinal[] = {20.0, -10.5, -9.5}; // {x , y, z}

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
const int DIAGONAL_MOVE_TIME = 2000;

const float L1 = 11.3;
const float L2 = 8;
const float L3 = 18;

unsigned long timeStamp;
bool pauseFlag = true;

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);

void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial.println("I'm in setup");

  shoulder.attach(pinShoulder);
  elbow.attach(pinElbow);
  wrist.attach(pinWrist);

  shoulder.writeMicroseconds(1532);
  elbow.writeMicroseconds(1525);
  wrist.writeMicroseconds(1535);
  delay(1000);

  // Ramp Calibration
  xRamp.go(L2 + L3, 500, LINEAR, ONCEFORWARD);
  yRamp.go(0.0, 500, LINEAR, ONCEFORWARD);
  zRamp.go(0.0, 500, LINEAR, ONCEFORWARD);

  while (xRamp.update() != L2 + L3 || yRamp.update() != 0.0 ||
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
    s2Width = moveServo(elbow, 1525, s2Angle);
    s3Width = moveServo(wrist, 1535, s3Angle);

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
    s2Width = moveServo(elbow, 1525, s2Angle);
    s3Width = moveServo(wrist, 1535, s3Angle);

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

    if (millis() - timeStamp >= DIAGONAL_MOVE_TIME / 2 && pauseFlag) {
      xRamp.pause();
      yRamp.pause();
      zRamp.pause();

      // MOVE ROBOT BACK HERE
      Serial.println("Paused");
      delay(3000);

      xRamp.resume();
      yRamp.resume();
      zRamp.resume();

      pauseFlag = false;
    }

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
    s2Width = moveServo(elbow, 1525, s2Angle);
    s3Width = moveServo(wrist, 1535, s3Angle);

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
