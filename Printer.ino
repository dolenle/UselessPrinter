/* 
 *  Useless Machine with Touch Sense
 */

#include <Encoder.h>
#include <MPR121.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <avr/interrupt.h> 

#define MOTOR_PWM 11
#define MOTOR_0 7
#define MOTOR_1 8
#define SR_LOAD 5
#define MICROSWITCH 4

#define MAX_PWM 255
#define ACCEL 0.7
#define NUM_SWITCHES 5
#define LID_DELAY 500
#define MARGIN 50

enum fingerSteps {FNG_REST, FNG_HOLD, FNG_PRESS};
enum lidSteps {LID_OPEN, LID_CLOSED};

unsigned int switchPos[] = {500,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000};
char touched[NUM_SWITCHES];
char touchPtr = -1;
char switched = -1;
char lastPressed;

boolean proximity;
boolean lidOpen;
unsigned long lidOpenTime;

int motorSpeed;
unsigned int carriagePos;
unsigned int fingerPos;

Encoder myEnc(2, 3);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  pinMode(SR_LOAD, OUTPUT);
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
  MPR121.setTouchThreshold(12,1);
  MPR121.setReleaseThreshold(20); 
  MPR121.setTouchThreshold(12,0 ); 
  MPR121.updateTouchData();
  MPR121.setProxMode(PROX0_11);

  TCCR2B = TCCR2B & 0xF8 | 0x1; //increase PWM frequency

  SPI.begin();
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
  }

  //read switch and touch
  MPR121.updateTouchData();
  for(char i=0; i<NUM_SWITCHES; i++) {
    if(MPR121.isNewTouch(i)) {
      touched[++touchPtr] = i; //push to stack
//      Serial.print(i);
//      Serial.println(" was just touched");  
    } else if(MPR121.isNewRelease(i)) {
      touchPtr--;
    }
  }
  //read proximity
  if(MPR121.isNewTouch(12)) {
    proximity = true;
//    Serial.println("Proximity"); 
  } else if(MPR121.isNewRelease(12)){
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
  }

  //If switch was sucessfully pressed, stop pressing it
  if(fingerPos == FNG_PRESS && switched != lastPressed) {
    fingerPos = FNG_HOLD;
  }

  //Get next carriage position
  if(switched >= 0) {
    carriagePos = switchPos[switched];
  } else if(touchPtr >= 0) {
    carriagePos = switchPos[touched[touchPtr]];
  }

  //Get next finger position
  if(switched >= 0 && now - lidOpenTime > LID_DELAY && error < MARGIN) {
    fingerPos = FNG_PRESS; //press switch
    lastPressed = switched;
  } else if(touchPtr >= 0 && now - lidOpenTime > LID_DELAY && error < MARGIN) {
    fingerPos = FNG_HOLD; //hover over switch
    Serial.println("Hover");
  } else {
    fingerPos = FNG_REST; //retract
  }

  //Close lid if no input
  if(lidOpen && !proximity && fingerPos == FNG_REST && now - lidOpenTime > LID_DELAY) {
    Serial.println("Close lid");
    lidOpen = false;
  }
}

void readSwitches() { //2 cascaded 74LS165
  SPI.beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SR_LOAD, HIGH);
  unsigned int val = SPI.transfer16(0);
  digitalWrite(SR_LOAD, LOW);
  SPI.endTransaction();
  for(char i=0; i<NUM_SWITCHES; i++) {
    if(val & 0x01) {
      switched = i;
      break;
    } else {
      val >>= 1;
    }
  }
  switched = -1;
}

