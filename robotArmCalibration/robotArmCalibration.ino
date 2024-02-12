// import the servo library:
#include <Servo.h>

// create an object for a servo:
Servo S1;
Servo S2;
Servo S3;

// define the analog input pin that the potentiometer is 
// connected to, and the PWM output that the servo is 
// connected to:
int P1_pot = 0;
int S1_pwm = 9;
int S2_pwm = 10;
int S3_pwm = 11;

// declare variables, for use later:
int P1_val;
int S1_angle;
int S2_angle;
int S3_angle;

// initialise the servo and the serial monitor:
void setup() {
  S1.attach(S1_pwm);  
  S2.attach(S2_pwm);
  S3.attach(S3_pwm);
  Serial.begin(9600);
}
 
void loop() {
  // read the potentiometer value
  // (values will be between 0 and 1023):
  P1_val = analogRead(P1_pot);
  
  // scale the potentiometer value to a servo angle 
  // (angles between 0 and 180 degrees can be applied):
  S1_angle = 1503;
  S2_angle = 1415;
  S3_angle = 1345;

  // write the servo angle to the servo:
  S1.writeMicroseconds(S1_angle);
  S2.writeMicroseconds(S2_angle);
  S3.writeMicroseconds(S3_angle);

  // send info to the serial monitor
  // to inform of what has just happened:
  Serial.print("Servo Angle = ");
  Serial.print(S1_angle);
  Serial.println(" degrees.");

  // give the servo some time to respond (milliseconds):
  delay(15);
}
