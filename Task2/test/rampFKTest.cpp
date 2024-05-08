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

int s1Angle;
int s2Angle;
int s3Angle;

rampInt s1Ramp;
rampInt s2Ramp;
rampInt s3Ramp;

int angles[4][3] = {
    {97, 134, 115}, {83, 125, 131}, {83, 136, 150}, {71, 106, 127}};

void setup() {
  Serial.begin(9600);
  Serial.println("I'm in setup");

  shoulder.attach(pinShoulder);
  elbow.attach(pinElbow);
  wrist.attach(pinWrist);

  shoulder.write(angles[0][0]);
  elbow.write(angles[0][1]);
  wrist.write(angles[0][2]);

  // Ramp Calibration
  s1Ramp.go(angles[0][0], 500, LINEAR, ONCEFORWARD);
  s2Ramp.go(angles[0][1], 500, LINEAR, ONCEFORWARD);
  s3Ramp.go(angles[0][2], 500, LINEAR, ONCEFORWARD);

  while (s1Ramp.update() != angles[0][0] || s2Ramp.update() != angles[0][1] ||
         s3Ramp.update() != angles[0][2]) {
    s1Ramp.update();
    s2Ramp.update();
    s3Ramp.update();
  }

  for (int i = 1; i < 4; i++) {

    Serial.println("\nNext Position Movement");

    s1Ramp.go(angles[i][0], 2000, LINEAR, ONCEFORWARD);
    s2Ramp.go(angles[i][1], 2000, LINEAR, ONCEFORWARD);
    s3Ramp.go(angles[i][2], 2000, LINEAR, ONCEFORWARD);

    s1Angle = s1Ramp.update();
    s2Angle = s2Ramp.update();
    s3Angle = s3Ramp.update();

    do {
      shoulder.write(s1Angle);
      elbow.write(s2Angle);
      wrist.write(s3Angle);

      Serial.print("S1: ");
      Serial.print(s1Angle);
      Serial.print("\tS2: ");
      Serial.print(s2Angle);
      Serial.print("\tS3: ");
      Serial.println(s3Angle);

      s1Angle = s1Ramp.update();
      s2Angle = s2Ramp.update();
      s3Angle = s3Ramp.update();
      delay(20);
    } while (s1Angle != angles[i][0] || s2Angle != angles[i][1] ||
             s3Angle != angles[i][2]);
  }
}

void loop() {}