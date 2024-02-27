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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //Set all pins to input mode 
  pinMode(0,INPUT);
  pinMode(1,INPUT);
  pinMode(2,INPUT);

  pinMode(pinAI1R, OUTPUT);
  pinMode(pinAI2R, OUTPUT);
  pinMode(pinPWMAR, OUTPUT);

  pinMode(pinBI1L, OUTPUT);
  pinMode(pinBI2L, OUTPUT);
  pinMode(pinPWMBL, OUTPUT);

  pinMode(pinStandBy, OUTPUT);
  standBy = true;
  digitalWrite(pinStandBy, standBy);
  AI1R = false;
  AI2R = true;

  BI1L = true;
  BI2L = false;
}

void loop() {
  // put your main code here, to run repeatedly:

    digitalWrite(pinAI1R, AI1R);
    digitalWrite(pinAI2R, AI2R);
    digitalWrite(pinBI1L, BI1L);
    digitalWrite(pinBI2L, BI2L);
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
    analogWrite(pinPWMBL,50);
    // analogWrite(pinPWMAL, 120);
    // analogWrite(pinPWMBR, 120);
    if(CSreading > 600){
      analogWrite(pinPWMAR, 120);
      analogWrite(pinPWMBL, 120);
    }
    else if(RSreading < 400){     
      analogWrite(pinPWMAR, 120);
      analogWrite(pinPWMBL, 50);

    }
    else if(LSreading < 400){
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 120);
    }
    else{
      analogWrite(pinPWMAR, 50);
      analogWrite(pinPWMBL, 50);
    }
}
