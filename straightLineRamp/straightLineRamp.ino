// import the required libraries
#include <math.h>
#include <Servo.h>
#include <Ramp.h>

// create an object for a servo:
Servo s1;
Servo s2;
Servo s3;

int s1PWM = 9;
int s2PWM = 10;
int s3PWM = 11;

// declare variables, for use later:
float s1Angle;
// float s1AnglePrev;
int s1Width;
float s2Angle;
// float s2AnglePrev;
int s2Width;
float s3Angle;
// float s3AnglePrev;
int s3Width;
float coordinatesInitial[] = {24, 12, 0.0}; // {x, y, z}
float coordinatesFinal[] = {27, -5, 0.0}; // {x, y, z}
float xCurr;
float yCurr;
float zCurr; 

const float L1 = 11.3;
const float L2 = 8;
const float L3 = 22.5;

unsigned long timeStamp;
unsigned long elapsedTime;
unsigned long controlRate = 30; // milliseconds
// float radIncrementMax = (PI/3.0)*(controlRate/1000.0);
float radIncrementMax = 0.1;
float yPositionIncrement = (coordinatesFinal[1]-coordinatesInitial[1]) / 30;
float xPositionIncrement = (coordinatesFinal[0]-coordinatesInitial[0]) / 30;

class Interpolation{
  rampInt myRamp;
  int interpolationFlag = 0;
  int savedValue;

  public:
    int move(int input, int duration){
      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }
      int output = myRamp.update();               
      return output;
    }
};

Interpolation interpX;
Interpolation interpY;

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);
float rateLimiter(float targetAngle, float currentAngle, float increment);

// initialise the servo and the serial monitor:
void setup() {
  timeStamp = millis();

  s1.attach(s1PWM);  
  s2.attach(s2PWM);
  s3.attach(s3PWM);

  xCurr = 0;
  yCurr = 0;
  zCurr = 0;

  s1Angle = 0;
  s3Angle = -acos((sq(xCurr) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
  s2Angle = atan2(yCurr, xCurr) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));
  s1Width = moveServo(s1, 1550, s1Angle);
  s2Width = moveServo(s2, 1415, s2Angle);
  s3Width = moveServo(s3, 1345, s3Angle);

  delay(3000);
  
  Serial.begin(9600);


  // Calculating required S2 and S3 angles for the range of vertical line co ordinates (Up and Down)
  while (xCurr < coordinatesFinal[0]){
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

      xCurr = interpX.move(coordinatesFinal[0], 1000);
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    }
  }

  delay(1000);
  
  while (yCurr > coordinatesFinal[1]){
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

      yCurr = interpY.move(coordinatesFinal[1], 1000);
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    }
  }

  // delay(1000);

  // while (xCurr > coordinatesInitial[0]){
  //   s3Angle = -acos((sq(xCurr) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
  //   s2Angle = atan2(yCurr, xCurr) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));
    
  //   // give the servo some time to respond (milliseconds):
  //   elapsedTime = millis() - timeStamp;
  //   if (elapsedTime >= controlRate) {
      
  //     // anything here will execute at the "controlRate"  
      
  //     // write the servo angle to the servo:
  //     s1Width = moveServo(s1, 1550, s1Angle);
  //     s2Width = moveServo(s2, 1415, s2Angle);
  //     s3Width = moveServo(s3, 1345, s3Angle);

  //     xCurr -= xPositionIncrement;
  //     Serial.println(xCurr);
  
  //     // update the timestamp each time this executes:
  //     timeStamp = millis();
  //   }
  // }
}
 
void loop() {
  
}

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle){
  float minPulseWidth = float(refPulseWidth)-1000.0;
  int cmdSignal = (servoRadAngle+(PI/2))*(2000.0/PI)+minPulseWidth;
  servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}

float rateLimiter(float targetAngle, float currentAngle, float increment){
  return;
}
