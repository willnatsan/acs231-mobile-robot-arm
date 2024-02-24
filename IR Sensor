int IRAnalogPin = 1;
int IRReading;
 
void setup() {
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
  pinMode(LEDpin, OUTPUT);
}
 
void loop() {
  IRReading = analogRead(IRAnalogPin);
  Serial.print("IR reading = ");
  Serial.println(IRReading); 
  delay(250);
}
