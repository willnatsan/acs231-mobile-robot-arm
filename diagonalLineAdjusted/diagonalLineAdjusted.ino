// import the servo library:
#include <Servo.h>
#include <math.h>

// create an object for a servo:
Servo s1;
Servo s2;
Servo s3;

int s1PWM = 9;
int s2PWM = 10;
int s3PWM = 11;

// declare variables, for use later:
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
float coordinatesInitial[] = {17.2, 10.0, 10.0}; // {x, y, z}
float coordinatesFinal[] = {17.2, -5.0, -8.0}; // {x, y, z}
float xCurr;
float yCurr;
float zCurr;

const float L1 = 11.3;
const float L2 = 8;
const float L3 = 13.5;

unsigned long timeStamp;
unsigned long elapsedTime;
unsigned long controlRate = 30; // milliseconds
// float radIncrementMax = (PI/3.0)*(controlRate/1000.0);
float radIncrementMax = 0.1;
float yPositionIncrement = (coordinatesFinal[1]-coordinatesInitial[1]) / 30;
float zPositionIncrement = (coordinatesFinal[2]-coordinatesInitial[2]) / 30;

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);
//int moveServoSlow(VarSpeedServo servo, float servoRadAngle);
//float rateLimiter(float targetAngle, float currentAngle, float increment);

// initialise the servo and the serial monitor:
void setup() {
  delay(500);
  Serial.begin(9600);
  timeStamp = millis();
  
  s1.attach(s1PWM);  
  s2.attach(s2PWM);
  s3.attach(s3PWM);
  Serial.println("I'm in setup");
  xCurr = coordinatesInitial[0];
  yCurr = coordinatesInitial[1];
  zCurr = coordinatesInitial[2];
  s1Angle = atan2(zCurr,xCurr);
  s3Angle = -acos((sq(xCurr) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
  s2Angle = atan2(yCurr, xCurr) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));  
  s1Width = moveServo(s1, 1550,s1Angle);
  s2Width = moveServo(s2, 1415, s2Angle);
  s3Width = moveServo(s3, 1345,s3Angle);
  delay(2000);
  
  
}
 
void loop() {
  // Calculating required S2 and S3 angles for the range of vertical line co ordinates (Up and Down)
  
  while (yCurr >= coordinatesFinal[1] && zCurr >= coordinatesFinal[2]){
    s1Angle = atan2(zCurr,xCurr);
    xCurr = sqrt(sq(zCurr)+sq(xCurr)); // Update xCurr value with rotating frame of reference
    s3Angle = -acos((sq(xCurr) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
    s2Angle = atan2(yCurr, xCurr) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));
    
    // give the servo some time to respond (milliseconds):
    elapsedTime = millis() - timeStamp;
    if (elapsedTime >= controlRate) {
      
      // anything here will execute at the "controlRate"  
      
      // write the servo angle to the servo:
      s1Width = moveServo(s1, 1550, s1Angle);
      s2Width = moveServo(s2, 1415, s2Angle);
      s3Width = moveServo(s3, 1345, s3Angle);

      yCurr += yPositionIncrement;
      zCurr += zPositionIncrement;
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    }
  }

//  while (yCurr <= coordinatesInitial[1]){
//    s1Angle = 0;
//    s3Angle = acos((sq(xCurr) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
//    s2Angle = -(atan2(yCurr, xCurr) - atan2(L3*sin(-s3Angle), L2+L3*cos(-s3Angle)));
//    
//    // give the servo some time to respond (milliseconds):
//    elapsedTime = millis() - timeStamp;
//    if (elapsedTime >= controlRate) {
//      
//      // anything here will execute at the "controlRate"
//  
//      // write the servo angle to the servo:
//      s1Width = moveServo(s1, 1350, s1Angle);
//      s2Width = moveServo(s2, 1771, s2Angle);
//      s3Width = moveServo(s3, 1512, s3Angle);
//
//      yCurr -= yPositionIncrement;
//  
//      // update the timestamp each time this executes:
//      timeStamp = millis();
//    }
//  }
}

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle){
  float minPulseWidth = float(refPulseWidth)-1000.0;
  int cmdSignal = (servoRadAngle+(PI/2))*(2000.0/PI)+minPulseWidth;
  servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}
