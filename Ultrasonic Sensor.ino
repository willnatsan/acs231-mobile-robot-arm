int ledPin = 5;

const int trigPin = 9;  //Used to transmit ultrasonic signals
const int echoPin = 10; //Used to receive ultrasonic signals

float duration, distance;  

void setup() {  
  pinMode(ledPin, OUTPUT);
	pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 
  digitalWrite(ledPin,LOW); 
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
    digitalWrite(ledPin,HIGH);
  }
  else{
    digitalWrite(ledPin,LOW);
  }

  Serial.print("Distance: ");  
	Serial.println(distance);  
	delay(100);  
}  
