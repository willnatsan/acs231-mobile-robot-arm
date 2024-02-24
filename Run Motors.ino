/***MAKE SURE PWM PINS ARE CONNECTED ONLY TO ~ ONES ON ARDUINO

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
 
void setup(void) {
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
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
 
void loop(void) {
 
  // we'll need to change the range from the analog reading (0-1023) down to the range
  // used by analogWrite (0-255) with map!
  //LEDbrightness = map(potReading*1, 0, 1023, 0, 255);
    AI1L = true;
    AI2L = false;

    BI1R = false;
    BI2R = true;
  // digitalWrite(pinAI1L, AI1L);
  // digitalWrite(pinAI2L, AI2L);
  // Serial.print("PWM = ");
  // Serial.println(pwmValueL);
  // analogWrite(pinPWMAL, pwmValueL);

  digitalWrite(pinBI1R, BI1R);
  digitalWrite(pinBI2R, BI2R);
  Serial.print("PWM = ");
  Serial.println(pwmValueR);
  analogWrite(pinPWMBR, pwmValueR);
  // LED gets brighter the harder you press
  //analogWrite(LEDpin, LEDbrightness); // here you are using PWM !!!
}



