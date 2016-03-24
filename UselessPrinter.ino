/* 
 *  Useless Machine with Touch Sense
 */

#include <Encoder.h>
#include <MPR121.h>
#include <Wire.h>
#include <Servo.h>
#include <avr/interrupt.h> 

//Pin assignments
#define MOTOR_PWM 11
#define MOTOR_0 7
#define MOTOR_1 8
#define SR_LOAD 5
#define SR_CLK 6
#define SR_DATA 12
#define MICROSWITCH 4
#define LID_SERVO 9
#define FINGER_SERVO 10

//Parameters
#define MAX_PWM 255
#define ACCEL 0.5 //higher -> more accuracy and overshoot
#define NUM_SWITCHES 12
#define LID_DELAY 500
#define MARGIN 50

enum fingerSteps {FNG_REST, FNG_HOLD, FNG_PRESS};
enum lidSteps {LID_OPEN=10, LID_CLOSED=120};

unsigned int switchPos[] = {500,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000};
char touchStack[NUM_SWITCHES];
char touchInd[NUM_SWITCHES];
char touchPtr = -1;
char switched = -1;
char lastPressed;

boolean proximity;
boolean lidOpen;
unsigned long lidOpenTime;

int motorSpeed;
unsigned int carriagePos;
unsigned int fingerPos;

Encoder myEnc(2, 3); //hardware interrupt
Servo lidServo;
Servo fingerServo;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  pinMode(SR_LOAD, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_DATA, INPUT);
  pinMode(MICROSWITCH, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_0, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  digitalWrite(MOTOR_0, HIGH);

  //Initialize position
  analogWrite(MOTOR_PWM, 64);
  while(digitalRead(MICROSWITCH) == HIGH); //wait
  digitalWrite(MOTOR_1, HIGH);
  delay(500); //wait to settle
  myEnc.write(0);
  while(myEnc.read() < 50) {
    digitalWrite(MOTOR_0, LOW);
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
  
  MPR121.setTouchThreshold(80);
  MPR121.setTouchThreshold(12,8);
  MPR121.setReleaseThreshold(20); 
  MPR121.setReleaseThreshold(12,5); 
  MPR121.updateTouchData();
  MPR121.setProxMode(PROX0_11);

  TCCR2B = TCCR2B & 0xF8 | 0x1; //increase PWM frequency

  //initialize servos
  lidServo.attach(LID_SERVO);
  fingerServo.attach(FINGER_SERVO);
  lidServo.write(LID_CLOSED);
  fingerServo.write(FNG_REST);
}

void loop() {
  //Update motor
  int error = carriagePos - myEnc.read();
  motorSpeed = ACCEL*error;
  if(motorSpeed >= 0) {
    analogWrite(MOTOR_PWM, min(motorSpeed, MAX_PWM));
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
  } else if(motorSpeed < 0) {
    analogWrite(MOTOR_PWM, -max(motorSpeed, -MAX_PWM));
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    error = -error;
  }

  //read switch and touch
  MPR121.updateTouchData();
  for(char i=0; i<NUM_SWITCHES; i++) {
    if(MPR121.isNewTouch(i)) {
      touchInd[i] = 1;
      touchStack[++touchPtr] = i; //push to stack
//      Serial.print(i);
//      Serial.println(" was just touched");  
    } else if(MPR121.isNewRelease(i)) {
      if(touchStack[touchPtr] == i) {
        touchPtr--;
      }
      touchInd[i] = 0;
    }
  }
  //read proximity
  if(MPR121.isNewTouch(12)) {
    proximity = true;
//    Serial.println("Proximity"); 
  } else if(MPR121.isNewRelease(12)) {
//    Serial.println("No Proximity");
    proximity = false;
  }
  readSwitches();

  unsigned long now = millis();

  //First, open lid if touch or switched on
  if(!lidOpen && (touchPtr >= 0 || switched >= 0 || proximity)) {
    //open lid
    Serial.println("Open Lid");
    lidOpen = true;
    lidOpenTime = now;
    lidServo.write(LID_OPEN);
  }

  //If switch was sucessfully pressed, stop pressing it
  if(fingerPos == FNG_PRESS && switched != lastPressed) {
    fingerServo.write(fingerPos = FNG_HOLD);
  }

  //Get next carriage position
  if(switched >= 0) {
    carriagePos = switchPos[switched];
  } else if(touchPtr >= 0) {
    if(touchInd[touchStack[touchPtr]]) {
      carriagePos = switchPos[touchStack[touchPtr]];
    } else {
      touchPtr--;
    }
  }

  //Get next finger position
  if(switched >= 0 && now - lidOpenTime > LID_DELAY) {
    if(error < MARGIN) {
      fingerServo.write(fingerPos = FNG_PRESS); //press switch
      lastPressed = switched;
    } else {
      fingerServo.write(fingerPos = FNG_HOLD); //wait to move to next switch
    }
  } else if(touchPtr >= 0 && now - lidOpenTime > LID_DELAY && error < MARGIN) {
    fingerServo.write(fingerPos = FNG_HOLD); //hover over switch
    Serial.println("Hover");
  } else {
    fingerServo.write(fingerPos = FNG_REST); //retract
  }

  //Close lid if no input
  if(lidOpen && !proximity && fingerPos == FNG_REST && now - lidOpenTime > LID_DELAY) {
    Serial.println("Close lid");
    lidOpen = false;
    lidServo.write(LID_CLOSED);
  }
}

void readSwitches() { //2 cascaded 74LS165, use SER as LSB
  digitalWrite(SR_LOAD, HIGH);
  unsigned int val = (shiftIn(SR_DATA, SR_CLK, MSBFIRST) << 8) | shiftIn(SR_DATA, SR_CLK, MSBFIRST);
  digitalWrite(SR_LOAD, LOW);
  for(char i=0; i<NUM_SWITCHES; i++) {
    if(val & 0x01) {
      switched = i;
      return;
    } else {
      val >>= 1;
    }
  }
  switched = -1;
}

