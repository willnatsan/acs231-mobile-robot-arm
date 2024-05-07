/*
 * ThreeServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 3 servos synchronously.
 *  Demonstrates the use of ServoEasingArray and ServoEasingNextPositionArray.
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 3
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo.
//#define DEBUG                              // Activating this enables generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.
#include "ServoEasing.hpp"
#include "PinDefinitionsAndMore.h"

/*
 * Pin mapping table for different platforms - used by all examples
 *
 * Platform         Servo1      Servo2      Servo3      Analog     Core/Pin schema
 * -------------------------------------------------------------------------------
 * (Mega)AVR + SAMD    9          10          11          A0
 * ATtiny3217         20|PA3       0|PA4       1|PA5       2|PA6   MegaTinyCore
 * ESP8266            14|D5       12|D6       13|D7        0
 * ESP32               5          18          19          A0
 * BluePill          PB7         PB8         PB9         PA0
 * APOLLO3            11          12          13          A3
 * RP2040             6|GPIO18     7|GPIO19    8|GPIO20
 */

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

#define START_DEGREE_VALUE  90 // The degree value written to the servo at time of attach.
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
// float coordinatesInitial[] = {14.0, 7.0, 8.0}; // {x, y, z}
// float coordinatesFinal[] = {14.0, -4.0, -4.0}; // {x, y, z}
float coordinatesInitial[] = {22.0, 10, 9.5}; // {x, y, z}
float coordinatesFinal[] = {23.0, -9, -9.5}; // {x, y, z}
float xCurr;
float xCurrNew;
float yCurr;
float zCurr;

const float L1 = 11.3;
const float L2 = 8;
// const float L3 = 9.5;
const float L3 = 20.2;
unsigned long timeStamp;
unsigned long elapsedTime;
unsigned long controlRate = 30; // milliseconds
// float radIncrementMax = (PI/3.0)*(controlRate/1000.0);
float radIncrementMax = 0.1;
float yPositionIncrement = (coordinatesFinal[1]-coordinatesInitial[1]) / 30;
float zPositionIncrement = (coordinatesFinal[2]-coordinatesInitial[2]) / 30;
float xPositionIncrement = (coordinatesFinal[0]-coordinatesInitial[0]) / 30;
int moveServo(Servo servo, int refPulseWidth, float servoRadAngle);

void blinkLED();

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));
#endif

    /************************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *
     * The order of the attach() determine the position
     * of the Servos in internal ServoEasing::ServoEasingArray[]
     ***********************************************************/
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
#endif
    Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO2_PIN)));
#endif
    Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);

    /*
     * Check at least the last call to attach()
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO3_PIN)));
#endif
    if (Servo3.attach(SERVO3_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(
                F("Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
        while (true) {
            blinkLED();
        }
    }

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    /*
     * Print internal servo control data
     */
    Servo1.print(&Serial);
    Servo2.print(&Serial);
    ServoEasing::ServoEasingArray[2]->print(&Serial); // "ServoEasing::ServoEasingArray[2]->" can be used instead of "Servo3."
#endif

#if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Legend for Arduino plotter
    Serial.println(); // end of line of attach values
    Serial.println(F("Servo1, Servo2, Servo3"));
#endif

    // Wait for servos to reach start position.
    delay(1000);
    Serial.println(F("Start loop"));
    timeStamp = millis();
    uint16_t tSpeed = 20;

    xCurr = coordinatesInitial[0];
    yCurr = coordinatesInitial[1];
    zCurr = coordinatesInitial[2];
    s1Angle = atan2(zCurr,xCurr);
    xCurrNew = sqrt(sq(zCurr)+sq(xCurr));
    s3Angle = -acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
    s2Angle = atan2(yCurr, xCurrNew) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));  
    s1Width = moveServo(Servo1, 1575, s1Angle);
    s2Width = moveServo(Servo2, 1500, s2Angle);
    s3Width = moveServo(Servo3, 1550, s3Angle);
    //Servo1.setEasingType(EASE_LINEAR); // EASE_LINEAR is default

    Servo1.setEaseTo(s1Width, 30);
    Servo2.setEaseTo(s2Width, 30);
    Servo3.startEaseTo(s3Width, 30); // Start interrupt for all servos. No synchronization here sinc
    delay(500);


}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {
    uint16_t tSpeed = 20;
    while (xCurr < coordinatesFinal[0]){
    s1Angle = atan2(zCurr,xCurr);
    xCurrNew = sqrt(sq(zCurr)+sq(xCurr));
    s3Angle = -acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
    s2Angle = atan2(yCurr, xCurrNew) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));  
    Serial.print("Base Angle = ");
    Serial.print(s1Angle*57.2);
    Serial.print("\tElbow Angle = ");
    Serial.print(s2Angle*57.2);
    Serial.print("\tWrist Angle = ");
    Serial.println(s3Angle*57.2);    
    // give the servo some time to respond (milliseconds):
    elapsedTime = millis() - timeStamp;
    // if (elapsedTime >= controlRate) {
      
      // anything here will execute at the "controlRate"  
      
      // write the servo angle to the servo:
      s1Width = moveServo(Servo1, 1575, s1Angle);
      s2Width = moveServo(Servo2, 1500, s2Angle);
      s3Width = moveServo(Servo3, 1550, s3Angle);
      Servo1.setEaseTo(s1Width, 30);
    Servo2.setEaseTo(s2Width, 30);
    Servo3.startEaseTo(s3Width, 30); // Start interrupt for all servos. No synchronization here sinc

      xCurr += xPositionIncrement;
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    
  }


  delay(2000);
  
  while (yCurr >= coordinatesFinal[1] && zCurr >= coordinatesFinal[2]){
    s1Angle = atan2(zCurr,xCurr);
    xCurrNew = sqrt(sq(zCurr)+sq(xCurr)); // Update xCurr value with rotating frame of reference
    s3Angle = -acos((sq(xCurrNew) + sq(yCurr) - sq(L2) - sq(L3))/(2*L2*L3));
    s2Angle = atan2(yCurr, xCurrNew) - atan2(L3*sin(s3Angle), L2+L3*cos(s3Angle));

    // give the servo some time to respond (milliseconds):
    // elapsedTime = millis() - timeStamp;
    // if (elapsedTime >= controlRate) {
      
      // anything here will execute at the "controlRate"  
      
      // write the servo angle to the servo:
      s1Width = moveServo(Servo1, 1575, s1Angle);
      s2Width = moveServo(Servo2, 1500, s2Angle);
      s3Width = moveServo(Servo3, 1550, s3Angle);
      Servo1.setEaseTo(s1Width, 20);
    Servo2.setEaseTo(s2Width, 20);
    Servo3.startEaseTo(s3Width, 20); // Start interrupt for all servos. No synchronization here sinc
          Serial.print("Base Angle = ");
        Serial.print(s1Width);
        Serial.print("\tElbow Angle = ");
        Serial.print(s2Width);
        Serial.print("\tWrist Angle = ");
        Serial.println(s3Width);  

      yCurr += yPositionIncrement;
      zCurr += zPositionIncrement;
  
      // update the timestamp each time this executes:
      timeStamp = millis();
    
    }

    /*
     * Move all 3 servos independently
     */



// #if !defined(PRINT_FOR_SERIAL_PLOTTER)
//     Serial.println(F("Move independently to 0/0/0 degree with 80/40/20 degree per second using interrupts"));
// #endif
//     Servo1.setEaseTo(1500, 40);
//     Servo2.setEaseTo(2000, 40);
//     Servo3.startEaseTo(2000, 40); // Start interrupt for all servos. No synchronization here since the servos should move independently.
//     delay(2000);
//     Servo1.setEaseTo(1500, 40);
//     Servo2.setEaseTo(1500, 40);
//     Servo3.startEaseTo(1500, 40); // Start interrupt for all servos. No synchronization here sinc
    
//     //Blink until servos stops
//     while (ServoEasing::areInterruptsActive()) {
//         blinkLED();
//     }

//     delay(2000);
}

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle){
  float minPulseWidth = float(refPulseWidth)-1000.0;
  int cmdSignal = (servoRadAngle+(PI/2))*(2000.0/PI)+minPulseWidth;
  //servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}
