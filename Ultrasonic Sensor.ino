int ledPin = 5;

const int trigPin = 9;  
const int echoPin = 10; 

float duration, distance;  

void setup() {  
  pinMode(ledPin, OUTPUT);
	pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 
  digitalWrite(ledPin,LOW); 
	Serial.begin(9600);  
}  

void loop() {  
	digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);  

  duration = pulseIn(echoPin, HIGH);  
  distance = (duration*.0343)/2; 

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
