// import the servo library:
#include <Servo.h>

// create an object for a servo:
Servo S1;

// define the analog input pin that the potentiometer is 
// connected to, and the PWM output that the servo is 
// connected to:
int P1_pot = 0;
int S1_pwm = 9;

// declare variables, for use later:
int P1_val;
int S1_angle;

// initialise the servo and the serial monitor:
void setup() {
  S1.attach(S1_pwm);  
  Serial.begin(9600);
}
 
void loop() {
  // read the potentiometer value
  // (values will be between 0 and 1023):
  P1_val = analogRead(P1_pot);
  
  // scale the potentiometer value to a servo angle 
  // (angles between 0 and 180 degrees can be applied):
  S1_angle = map(P1_val, 0, 1023, 0, 180);

  // write the servo angle to the servo:
  S1.write(S1_angle);

  // send info to the serial monitor
  // to inform of what has just happened:
  Serial.print("Servo Angle = ");
  Serial.print(S1_angle);
  Serial.println(" degrees.");

  // give the servo some time to respond (milliseconds):
  delay(15);
}
