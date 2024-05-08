// // import the servo library:
#include <Servo.h>
#include <Arduino.h>
#include <ArduinoSTL.h>

// create an object for a servo:
Servo S1;
Servo S2;
Servo S3;

int S3_pwm = A7; 
int S2_pwm = A6;
int S1_pwm = A5;

// declare variables, for use later:
int S1_angle;
int S2_angle;
int S3_angle;

// initialise the servo and the serial monitor:
void setup() {
  S1.attach(S1_pwm,615,2450);  
  S2.attach(S2_pwm,625,2425);
  S3.attach(S3_pwm,620,2450);
  Serial.begin(9600);
  //S1.write(1580);
  S1.write(90);
  S2.write(90);
  S3.write(90);

  //S3.write(90);
  delay(500);
}
 
void loop() {
  
}
