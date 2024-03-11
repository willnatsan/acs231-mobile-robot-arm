/***Left Motor Declarations***/
unsigned char pwmValueL = 125;
const int pinAI1R = 7;       // Pin allocation for AI1
const int pinAI2R = 8;       // Pin allocation for AI2
const int pinPWMAR = 5;       // Pin allocation for the PWM pin
boolean AI1R = 0;            // AI1 pin value
boolean AI2R = 0;            // AI2 pin value

/***Right Motor Declarations***/
unsigned char pwmValueR = 125;
const int pinBI1L = 10;       // Pin allocation for AI1
const int pinBI2L = 11;       // Pin allocation for AI2
const int pinPWMBL = 3;       // Pin allocation for the PWM pin
boolean BI1L = 0;            // AI1 pin value
boolean BI2L = 0;            // AI2 pin value

const int pinStandBy = 9;   // Pin allocation for the standby pin
boolean standBy = 0;        // standBy pin Value


boolean rotDirect = 0;      // Rotation direction variable


/**Ultrasonic Pins**/
const int trigPin = 52;
const int echoPin = 53;
// defines variables
long duration;
int distance;

//Declare and Initialize Analog Pins Numbers for Line Following Sensors
int LeftSensor = 0;
int CentreSensor = 1;
int RightSensor = 2;

//Declare and Initialize Sensor Reading Values
int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

void setup() {
  pinMode(0,INPUT);
  pinMode(1,INPUT);
  pinMode(2,INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  pinMode(pinAI1R, OUTPUT);
  pinMode(pinAI2R, OUTPUT);
  pinMode(pinPWMAR, OUTPUT);

  pinMode(pinBI1L, OUTPUT);
  pinMode(pinBI2L, OUTPUT);
  pinMode(pinPWMBL, OUTPUT);

  pinMode(pinStandBy, OUTPUT);
  standBy = true;
  digitalWrite(pinStandBy, standBy);

  //Spin
  AI1R = false;
  AI2R = true;

  BI1L = false;
  BI2L = true;
  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);
  analogWrite(pinPWMAR, 120);
  analogWrite(pinPWMBL, 120);
  delay(2700);
  
  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
  
  //End spin

  AI1R = false;
  AI2R = true;

  BI1L = true;
  BI2L = false;
}
void loop() {
  // Clears the trigPin
  do{
  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);
  

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  analogWrite(pinPWMAR, 100);
  analogWrite(pinPWMBL, 100);
 
  }while(distance > 15);

  
  analogWrite(pinPWMAR, 0);
  analogWrite(pinPWMBL, 0);
  delay(1500);
  
  do{
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
    LSreading = analogRead(LeftSensor);    
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);
    analogWrite(pinPWMAR, 80);
    analogWrite(pinPWMBL, 0);
  }while(LSreading < 600);

  do{
  LSreading = analogRead(LeftSensor);    
  CSreading = analogRead(CentreSensor);
  RSreading = analogRead(RightSensor);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);
  if(CSreading > 600 && LSreading > 600 && RSreading > 600){
      analogWrite(pinPWMAR, 120);
      analogWrite(pinPWMBL, 120);
      Serial.println("Going straight");
    }
    else if(RSreading < 400 && LSreading > 600){     
      analogWrite(pinPWMAR, 120);
      analogWrite(pinPWMBL, 100);
      Serial.println("Too much to right! Turning left");

    }
    else if(LSreading < 400 && RSreading > 600){
      analogWrite(pinPWMAR, 100);
      analogWrite(pinPWMBL, 120);
      Serial.println("Too much to left! Turning right");
    }
    else if(LSreading < 400 && CSreading < 400 & RSreading < 400){
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 50);
      Serial.println("Moving straight slow");
    }
  }while(distance > 10);
  while(1){
    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
  analogWrite(pinPWMAR, 0);
      analogWrite(pinPWMBL, 0);

  }
  
  
}
