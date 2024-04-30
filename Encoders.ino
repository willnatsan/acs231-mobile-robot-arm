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

#define PINA 18
#define PINB 19
#define PINC 20
#define PIND 21
#define ENC_K 12

volatile long enc_count_right;
volatile float enc_rev_right;
volatile long enc_count_left;
volatile float enc_rev_left;

unsigned long t0;

void setup(){
  pinMode(PINA,INPUT);
  pinMode(PINB,INPUT);
  attachInterrupt(digitalPinToInterrupt(PINA),channelA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINB),channelB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINC),channelC,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIND),channelD,CHANGE);
  pinMode(pinAI1R, OUTPUT);
  pinMode(pinAI2R, OUTPUT);
  pinMode(pinPWMAR, OUTPUT);

  pinMode(pinBI1L, OUTPUT);
  pinMode(pinBI2L, OUTPUT);
  pinMode(pinPWMBL, OUTPUT);
  pinMode(pinStandBy, OUTPUT);
  standBy = true;
  digitalWrite(pinStandBy, standBy);


  enc_count_right = 0;
  enc_rev_right = 0;
  enc_count_left = 0;
  enc_rev_left = 0;
  Serial.begin(9600);
  t0=millis();
  AI1R = false;
  AI2R = true;
  BI1L = false;
  BI2L = true;

}

void loop(){
  while(enc_rev_right <247 || enc_rev_left < 223 ){
  digitalWrite(pinAI1R, AI1R);
  digitalWrite(pinAI2R, AI2R);
  digitalWrite(pinBI1L, BI1L);
  digitalWrite(pinBI2L, BI2L);  
  analogWrite(pinPWMBL, 100);
  analogWrite(pinPWMAR, 100);

  if(millis()-t0>20){
    Serial.print("Encoder right count: ");
    Serial.print(enc_count_right);
    Serial.print("\tEncoder right revolution: ");
    Serial.print(enc_rev_right);

    Serial.print("\nEncoder left count: ");
    Serial.print(enc_count_left);
    Serial.print("\tEncoder left revolution: ");
    Serial.println(enc_rev_left);
  }
  }
   while(1){
       analogWrite(pinPWMAR, 0);
       analogWrite(pinPWMBL, 0);


  }
}

void channelA(){
  if(digitalRead(PINA)==HIGH){
    if(digitalRead(PINB==LOW)) enc_count_right++;
    else enc_count_right--;
  }
  else{
    if(digitalRead(PINB)==HIGH) enc_count_right++;
    else enc_count_right--;
  }
  enc_rev_right = (float)enc_count_right/ENC_K;
}

void channelB(){
  if(digitalRead(PINB)==HIGH){
    if(digitalRead(PINA==LOW)) enc_count_right++;
    else enc_count_right--;
  }
  else{
    if(digitalRead(PINA)==HIGH) enc_count_right++;
    else enc_count_right--;
  }
  enc_rev_right = (float)enc_count_right/ENC_K;
}

void channelC(){
  if(digitalRead(PINC)==HIGH){
    if(digitalRead(PIND==LOW)) enc_count_left++;
    else enc_count_left--;
  }
  else{
    if(digitalRead(PIND)==HIGH) enc_count_left++;
    else enc_count_left--;
  }
  enc_rev_left = (float)enc_count_left/ENC_K;
}

void channelD(){
  if(digitalRead(PIND)==HIGH){
    if(digitalRead(PINC==LOW)) enc_count_left++;
    else enc_count_left--;
  }
  else{
    if(digitalRead(PINC)==HIGH) enc_count_left++;
    else enc_count_left--;
  }
  enc_rev_left = (float)enc_count_left/ENC_K;
}