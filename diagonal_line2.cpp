// import the servo library:
#include <Servo.h>
#include <math.h>
#include <Arduino.h>
#include <ArduinoSTL.h>

// create an object for a servo:
Servo s1;
Servo s2;
Servo s3;

int s3_pwm = A7; 
int s2_pwm = A6;
int s1_pwm = A5;

// declare variables, for use later:
float s1Angle;
int s1Width;
float s2Angle;
int s2Width;
float s3Angle;
float s3AngleMax;
int s3Width;
// float coordinatesInitial[] = {14.0, 7.0, 8.0}; // {x, y, z}
// float coordinatesFinal[] = {14.0, -4.0, -4.0}; // {x, y, z}
float coordinatesInitial[] = {19.0, 12.5, 8.5}; // {x, y, z}
float coordinatesFinal[] = {21.0, -4.5, -8.5}; // {x, y, z}
float xCurr;
float xCurrNew;
float yCurr;
float zCurr;
unsigned long t0;

const float L1 = 11.5;
const float L2 = 8;
// const float L3 = 9.5;
const float L3 = 18;
unsigned long timeStamp;
unsigned long elapsedTime;
unsigned long controlRate = 30; // milliseconds
// float radIncrementMax = (PI/3.0)*(controlRate/1000.0);
float radIncrementMax = 0.1;
float yPositionIncrement = (coordinatesFinal[1]-coordinatesInitial[1]) / 20;
float zPositionIncrement = (coordinatesFinal[2]-coordinatesInitial[2]) / 20;
float xPositionIncrement = (coordinatesFinal[0]-coordinatesInitial[0]) / 20;
int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);

void setup() {
  delay(500);
  Serial.begin(9600);
  timeStamp = millis();
  t0 = millis();
  
  s1.attach(s1_pwm,615,2450);  
  s2.attach(s2_pwm,625,2425);
  s3.attach(s3_pwm,620,2450);
  Serial.println("I'm in setup");
  xCurr = coordinatesInitial[0];
  yCurr = coordinatesInitial[1];
  zCurr = coordinatesInitial[2];
  s1Angle = atan2(zCurr,xCurr);
  float n = sqrt(sq(xCurrNew) + sq(yCurr));
  s3Angle = PI - acos((sq(L2) + sq(L3) - sq(n)) / (2 * L2 * L3));
  s2Angle = atan2(yCurr, xCurrNew) + acos((sq(L2) + sq(n) - sq(L3)) /(2*L2*n));
  Serial.println(s1Angle*180/PI+90);
  Serial.println(s2Angle*180/PI-90);
  Serial.println(s3Angle*180/PI-90);

  delay(2000);

   Serial.println("\n\n\n\n");

}
void loop() {

  while (xCurr < coordinatesFinal[0]){
    s1Angle = atan2(zCurr,xCurr);
    xCurrNew = sqrt(sq(zCurr)+sq(xCurr));
    float n = sqrt(sq(xCurrNew) + sq(yCurr));
    s3Angle = PI - acos((sq(L2) + sq(L3) - sq(n)) / (2 * L2 * L3));
    s2Angle = atan2(yCurr, xCurrNew) + acos((sq(L2) + sq(n) - sq(L3)) /(2*L2*n));  
    
    // give the servo some time to respond (milliseconds):
    elapsedTime = millis() - timeStamp;
    if (elapsedTime >= controlRate) {
      
      // anything here will execute at the "controlRate"  
      
      s1.write(s1Angle*180/PI+90);
      s2.write(s2Angle*180/PI-90);
      s3.write(s3Angle*180/PI-90);

      xCurr += xPositionIncrement;
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    }
  }

  while(true){}
  delay(2000);
  
  while (yCurr >= coordinatesFinal[1] && zCurr >= coordinatesFinal[2]){
    s1Angle = atan2(zCurr,xCurr);
    xCurrNew = sqrt(sq(zCurr)+sq(xCurr)); 
    s3Angle = acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3)) / (2 * L2 * L3));
    s2Angle = atan2(yCurr, xCurrNew) - atan2(L3 * sin(-s3Angle), L2 + L3 * cos(-s3Angle));

    // give the servo some time to respond (milliseconds):
    elapsedTime = millis() - timeStamp;
    if (elapsedTime >= controlRate) {

      yCurr += yPositionIncrement;
      zCurr += zPositionIncrement;
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    }
    }
   
}

