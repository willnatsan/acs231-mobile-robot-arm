// import the servo library:
#include <Servo.h>

// create an object for a servo:
Servo S1;
Servo S2;
Servo S3;

int S1_pwm = 9;
int S2_pwm = 10;
int S3_pwm = 11;

// declare variables, for use later:
int S1_angle;
int S2_angle;
int S3_angle;

// initialise the servo and the serial monitor:
void setup() {
  S1.attach(S1_pwm);  
  S2.attach(S2_pwm);
  S3.attach(S3_pwm);
  Serial.begin(9600);

  delay(500);
}
 
void loop() {
  
  // scale the potentiometer value to a servo angle 
  // (angles between 0 and 180 degrees can be applied):
  S1_angle = 1550;
  S2_angle = 1415;
  S3_angle = 1345;

  // write the servo angle to the servo:
  S1.writeMicroseconds(S1_angle);
  S2.writeMicroseconds(S2_angle);
  S3.writeMicroseconds(S3_angle);

  // send info to the serial monitor
  // to inform of what has just happened:
  Serial.print("Base Angle = ");
  Serial.print(S1_angle);
  Serial.println(" rads.");
  Serial.println();
  Serial.print("Elbow Angle = ");
  Serial.print(S2_angle);
  Serial.println(" rads.");
  Serial.println();
  Serial.print("Wrist Angle = ");
  Serial.print(S3_angle);
  Serial.println(" rads.");
  Serial.println();
}
