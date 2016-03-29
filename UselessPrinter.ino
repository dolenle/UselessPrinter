/* 
 *  Useless Machine with Touch Sense
 *  Dolen Le 2016
 */

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <MPR121.h>
#include <Wire.h>
#include <Servo.h>

//Pin assignments
#define MOTOR_PWM 11
#define MOTOR_0 7
#define MOTOR_1 6
#define SR_LOAD 5 //PD5
#define SR_CLK 12 //PB4
#define SR_DATA 8 //PB0
#define SR_CLK_BIT 4
#define SR_DATA_BIT 0
#define MICROSWITCH 4
#define LID_SERVO 9
#define FINGER_SERVO 10
#define LED 13

//Parameters
#define MAX_PWM 255 //max motor speed
#define ACCEL 1.5 //higher -> more accuracy and overshoot
#define NUM_SWITCHES 12
#define LID_DELAY 100
#define PRESS_DELAY 100
#define MARGIN 30 //carriage position error margin

enum fingerSteps {FNG_REST=173, FNG_HOLD=120, FNG_PRESS=90}; //finger servo positions
enum lidSteps {LID_OPEN=10, LID_CLOSED=120}; //lid servo positions

unsigned int switchPos[] = {200,200,800,1400,2000,2600,3220,3800,4400,5000,5600,6200};
//unsigned int switchPos[] = {7000,6390,5782,5173,4564,3954,3345,2736,2127,1518,909,300}; //fliplr

char touchStack[NUM_SWITCHES];
char touchInd[NUM_SWITCHES];
char touchPtr = -1;
char switched = -1;
char lastPressed;

unsigned int switchVal;
boolean proximity;
boolean lidOpen;
unsigned long lidOpenTime;
unsigned long lastPressTime;

int motorSpeed;
unsigned int carriagePos = switchPos[0];
byte fingerPos;

Encoder carriageEnc(2, 3); //hardware interrupt
Servo lidServo;
Servo fingerServo;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(SR_LOAD, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_DATA, INPUT);
  pinMode(MICROSWITCH, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_0, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);

  //initialize servos
  lidServo.attach(LID_SERVO);
  fingerServo.attach(FINGER_SERVO);
  lidServo.write(LID_CLOSED);
  fingerServo.write(FNG_REST);

  //Initialize position
  digitalWrite(MOTOR_0, HIGH);
  analogWrite(MOTOR_PWM, 64);
  while(digitalRead(MICROSWITCH) == HIGH); //home
  digitalWrite(MOTOR_1, HIGH);
  delay(500); //wait to settle
  carriageEnc.write(0);
  while(carriageEnc.read() < 100) {
    digitalWrite(MOTOR_0, LOW); //ease off
  }
  digitalWrite(MOTOR_0, HIGH);
  delay(1000);

  Wire.begin();
  if(!MPR121.begin(0x5A)){
    Serial.println("MPR121 Error");  
    while(1);
  } else {
    Serial.println("Touch init");
  }

  MPR121.goFast(); //increase i2c frequency
  MPR121.setTouchThreshold(180);
  MPR121.setTouchThreshold(12,8);
  MPR121.setTouchThreshold(11,200);
  MPR121.setReleaseThreshold(60); 
  MPR121.setReleaseThreshold(12,5);
  MPR121.setProxMode(PROX0_11);

  TCCR2B = TCCR2B & 0xF8 | 0x1; //increase PWM frequency
  
  digitalWrite(LED, LOW);
}

void loop() {
  //Update motor
  int error = carriagePos - carriageEnc.read();
  motorSpeed = ACCEL*error;
  if(motorSpeed >= 0) {
    analogWrite(MOTOR_PWM, min(motorSpeed, MAX_PWM));
    digitalWrite(MOTOR_1, HIGH);
    digitalWrite(MOTOR_0, LOW);
  } else if(motorSpeed < 0) {
    analogWrite(MOTOR_PWM, -max(motorSpeed, -MAX_PWM));
    digitalWrite(MOTOR_0, HIGH);
    digitalWrite(MOTOR_1, LOW);
    error = -error;
  }

  //read touch
  MPR121.updateTouchData();
  for(char i=0; i<NUM_SWITCHES; i++) {
    if(MPR121.isNewTouch(i)) {
      touchStack[++touchPtr] = i; //push to stack
      touchInd[i] = touchPtr;
    } else if(MPR121.isNewRelease(i)) {
      for(char j=touchInd[i]; j<touchPtr; j++) {
         touchStack[j] = touchStack[j+1];
      }
      touchPtr--;
    }
  }
  //read proximity
  if(MPR121.isNewTouch(12)) {
    proximity = true;
  } else if(MPR121.isNewRelease(12)) {
    proximity = false;
  }
  //read switches
  readSwitches();

  unsigned long now = millis();

  //First, open lid if touch or switched on
  if(!lidOpen && (touchPtr >= 0 || switched >= 0 || proximity)) {
    lidOpen = true;
    lidOpenTime = now;
    lidServo.write(LID_OPEN);
    digitalWrite(LED, HIGH);
  }

  //If switch was sucessfully pressed, stop pressing it
  if(fingerPos == FNG_PRESS && switched != lastPressed) {
    fingerServo.write(fingerPos = FNG_HOLD);
    lastPressTime = now;
  }

  //Get next carriage position
  if(now-lastPressTime > PRESS_DELAY) {
    if(switched >= 0) {
      carriagePos = switchPos[switched];
    } else if(touchPtr >= 0) {
      carriagePos = switchPos[touchStack[touchPtr]];
    }
  }
  
  //Get next finger position
  if(switched >= 0 && now - lidOpenTime > LID_DELAY) {
    if(error < MARGIN && now-lastPressTime > PRESS_DELAY) {
      fingerServo.write(fingerPos = FNG_PRESS); //press switch
      lastPressed = switched;
    } else {
      fingerServo.write(fingerPos = FNG_HOLD); //wait to move to next switch
    }
  } else if(touchPtr >= 0 && now - lidOpenTime > LID_DELAY) {
    fingerServo.write(fingerPos = FNG_HOLD); //hover over switch
  } else {
    fingerServo.write(fingerPos = FNG_REST); //retract
  }

  //Close lid if no input
  if(lidOpen && !proximity && fingerPos == FNG_REST && now - lidOpenTime > LID_DELAY) {
    lidOpen = false;
    lidServo.write(LID_CLOSED);
    digitalWrite(LED, LOW);
  }
}

//faster version, approx 70uS
void readSwitches() {
  switchVal = 0;
  digitalWrite(SR_LOAD, HIGH);
  for (char i=15; i>=0; i--)  {
    PORTB ^= bit(SR_CLK_BIT); //clk high
    switchVal |= (bitRead(PINB, SR_DATA_BIT) << i);
    PORTB ^= bit(SR_CLK_BIT); //clk low
  }
  digitalWrite(SR_LOAD, LOW);
  for(char i=NUM_SWITCHES-1; i>=0; i--) {
    if(switchVal & 0x01) {
      switched = i;
      return;
    } else {
      switchVal >>= 1;
    }
  }
  switched = -1;
}

