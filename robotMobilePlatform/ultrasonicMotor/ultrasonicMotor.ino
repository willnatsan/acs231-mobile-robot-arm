int ledPin = 5;

const int trigPin = 52;  //Used to transmit ultrasonic signals
const int echoPin = 53; //Used to receive ultrasonic signals

float duration, distance;  

unsigned char pwmValueL = 125;
const int pinAI1L = 7;       // Pin allocation for AI1
const int pinAI2L = 8;       // Pin allocation for AI2
const int pinPWMAL = 5;       // Pin allocation for the PWM pin
boolean AI1L = 0;            // AI1 pin value
boolean AI2L = 0;            // AI2 pin value

/***Right Motor Declarations***/
unsigned char pwmValueR = 125;
const int pinBI1R = 10;       // Pin allocation for AI1
const int pinBI2R = 11;       // Pin allocation for AI2
const int pinPWMBR = 3;       // Pin allocation for the PWM pin
boolean BI1R = 0;            // AI1 pin value
boolean BI2R = 0;            // AI2 pin value

const int pinStandBy = 9;   // Pin allocation for the standby pin
boolean standBy = 0;        // standBy pin Value


boolean rotDirect = 0;      // Rotation direction variable

void setup() {  
  pinMode(ledPin, OUTPUT);
	pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 
  digitalWrite(ledPin,LOW); 

  pinMode(pinAI1L, OUTPUT);
  pinMode(pinAI2L, OUTPUT);
  pinMode(pinPWMAL, OUTPUT);

  pinMode(pinBI1R, OUTPUT);
  pinMode(pinBI2R, OUTPUT);
  pinMode(pinPWMBR, OUTPUT);

  pinMode(pinStandBy, OUTPUT);
  standBy = true;
  digitalWrite(pinStandBy, standBy);
  AI1L = false;
  AI2L = true;
  BI1R = true;
  BI2R = false;

	Serial.begin(9600);  
}  

void loop() {  
	digitalWrite(trigPin, LOW);  //Make sure signal is not yet transmitted
	delayMicroseconds(2);        //Wait for 2 microseconds
	digitalWrite(trigPin, HIGH); //Send out signal   
	delayMicroseconds(10);       //Wait for 10 microseconds
	digitalWrite(trigPin, LOW);  //Stop sending signals

  duration = pulseIn(echoPin, HIGH);  //echoPin goes high when signal bounces back and is received. Counting how long it takes for pin to go high
  distance = (duration*.0343)/2;      //Changing units to cm 

  if(distance <= 13.2){
    pwmValueL = 0;
    pwmValueR = 0;
  }
  else{
    pwmValueL = 125;
    pwmValueR = 125;
  }

  digitalWrite(pinAI1L, AI1L);
  digitalWrite(pinAI2L, AI2L);
  digitalWrite(pinBI1R, BI1R);
  digitalWrite(pinBI2R, BI2R);
  analogWrite(pinPWMAL, pwmValueL);
  analogWrite(pinPWMBR, pwmValueR);

  Serial.print("Distance: ");  
	Serial.println(distance);  
	delay(100);  
}  
