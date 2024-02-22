
unsigned char pwmValueL = 125;
const int pinAI1L = 7;       // Pin allocation for AI1
const int pinAI2L = 8;       // Pin allocation for AI2
const int pinPWMAL = 5;       // Pin allocation for the PWM pin

unsigned char pwmValueR = 125;
const int pinBI1R = 10;       // Pin allocation for AI1
const int pinBI2R = 11;       // Pin allocation for AI2
const int pinPWMBR = 12;       // Pin allocation for the PWM pin

boolean AI1L = 0;            // AI1 pin value
boolean AI2L = 0;            // AI2 pin value

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
  AI1L = false;
  AI2L = true;
  BI1R = true;
  BI2R = false;
}
 
void loop(void) {
    // AI1 = true;
    // AI2 = false;
    AI1L = false;
    AI2L = true;
    BI1R = true;
    BI2R = false;
  digitalWrite(pinAI1L, AI1L);
  digitalWrite(pinAI2L, AI2L);
  digitalWrite(pinBI1R, BI1R);
  digitalWrite(pinBI2R, BI2R);
  Serial.print("PWM = ");
  Serial.println(pwmValueL);
  analogWrite(pinPWMAL, pwmValueL);
  analogWrite(pinPWMBR, pwmValueR);
  // LED gets brighter the harder you press
  //analogWrite(LEDpin, LEDbrightness); // here you are using PWM !!!
 
  delay(500);
}









