/* 
Note: 
This is just a file I'm using to play around with some ideas for the
robot arm without messing with the current working code (interpolation,
different inverse kinematics, etc.). 
- William
*/

#include <Servo.h>
#include <Ramp.h>
#include <math.h>

// Define constants: Arm lengths & Reference pulsewidths
#define L1 11.3
#define L2 8
#define L3 17.2

// Initialise the servo objects and pins
Servo s1; // Base Servo
Servo s2; // Elbow Servo
Servo s3; // Wrist Servo
int s1PWM = 9;
int s2PWM = 10;
int s3PWM = 11;

// Initialise servo angle and pulsewidths variables
float s1Angle;
float s2Angle;
float s3Angle;
float s1Width;
float s2Width;
float s3Width;

// Initialise position variables
float coordinates[3][3] = {{18, 10, 0}, // {x, y, z}
                           {21, 10, 0}, // {x, y, z}
                           {21, -9, 0}} // {x, y, z}
float xCurr;
float xCurrNew; // Needed for z-axis movement
float yCurr;
float zCurr;

// Initialise control rate variables
unsigned long timeStamp;
unsigned long elapsedTime;
unsigned long controlRate = 30;
int incrementSteps = 80;
float xPositionIncrement;
float yPositionIncrement;
float zPositionIncrement;

// Defining class for interpolation (Makes impelmentation a bit easier with this library)
class Interpolation{
  rampInt position;
  bool newInterpolation = true;
  int savedValue;

  public:
    int update(int target, int duration){
      if (target != savedValue){
        newInterpolation = true;
      }
      savedValue = target;
      if (newInterpolation){
        position.go(target, duration, LINEAR, ONCEFORWARD)
        newInterpolation = false;
      }
      int output = position.update();
      return output;
    }
}

// Function prototypes
int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);
int posToAngle(float x, float y, float z);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
