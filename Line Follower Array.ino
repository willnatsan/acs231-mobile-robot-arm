/***Light: 0V - Dark ~653***/

//Declare and Initialize Pins Numbers for Sensors
int LeftSensor = 0;
int CentreSensor = 1;
int RightSensor = 2;

//Declare and Initialize Sensor Reading Values
int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //Set all pins to input mode 
  pinMode(0,INPUT);
  pinMode(1,INPUT);
  pinMode(2,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

    //Read values from sensors
    LSreading = analogRead(LeftSensor);
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);

    //Display sensor readings
    Serial.print("Left Sensor Reading = ");
    Serial.print(LSreading);
    Serial.print("\t");

    Serial.print("Centre Sensor Reading = ");
    Serial.print(CSreading);
    Serial.print("\t");

    Serial.print("Right Sensor Reading = ");
    Serial.println(RSreading);
}
