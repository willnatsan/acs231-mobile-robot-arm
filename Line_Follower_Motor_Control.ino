/***Light: 0V - Dark ~653***/

//Declare and Initialize Pins Numbers for Sensors
int LeftSensor = 0;
int CentreSensor = 1;
int RightSensor = 2;

//Declare and Initialize Sensor Reading Values
int LSreading = 0;
int CSreading = 0;
int RSreading = 0;

/***Left Motor Declarations***/
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
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //Set all pins to input mode 
  pinMode(0,INPUT);
  pinMode(1,INPUT);
  pinMode(2,INPUT);

  pinMode(pinAI1L, OUTPUT);
  pinMode(pinAI2L, OUTPUT);
  pinMode(pinPWMAL, OUTPUT);

  pinMode(pinBI1R, OUTPUT);
  pinMode(pinBI2R, OUTPUT);
  pinMode(pinPWMBR, OUTPUT);

  pinMode(pinStandBy, OUTPUT);
  standBy = true;
  digitalWrite(pinStandBy, standBy);
  AI1L = true;
  AI2L = false;

  BI1R = false;
  BI2R = true;
}

void loop() {
  // put your main code here, to run repeatedly:
    AI1L = true;
    AI2L = false;
    BI1R = false;
    BI2R = true;
    digitalWrite(pinAI1L, AI1L);
    digitalWrite(pinAI2L, AI2L);
    digitalWrite(pinBI1R, BI1R);
    digitalWrite(pinBI2R, BI2R);
    //Read values from sensors
    LSreading = analogRead(LeftSensor);    
    CSreading = analogRead(CentreSensor);
    RSreading = analogRead(RightSensor);
    Serial.print("Left Sensor Reading = ");
    Serial.print(LSreading);
    Serial.print("\t");

    Serial.print("Centre Sensor Reading = ");
    Serial.print(CSreading);
    Serial.print("\t");

    Serial.print("Right Sensor Reading = ");
    Serial.println(RSreading);

    // analogWrite(pinPWMAL, 120);
    // analogWrite(pinPWMBR, 120);
    if(CSreading > 600){
      analogWrite(pinPWMAL, 120);
      analogWrite(pinPWMBR, 120);
    }
    else if(LSreading>600 && RSreading < 400){     
      analogWrite(pinPWMAL, 75);
      analogWrite(pinPWMBR, 120);
    }
    else if(RSreading>600 && LSreading < 400){
      analogWrite(pinPWMAL, 125);
      analogWrite(pinPWMBR, 75);
    }
    else{
      analogWrite(pinPWMAL, 120);
      analogWrite(pinPWMBR, 120);
    }

    //Display sensor readings
    // Serial.print("Left Sensor Reading = ");
    // Serial.print(LSreading);
    // Serial.print("\t");

    // Serial.print("Centre Sensor Reading = ");
    // Serial.print(CSreading);
    // Serial.print("\t");

    // Serial.print("Right Sensor Reading = ");
    // Serial.println(RSreading);
}
