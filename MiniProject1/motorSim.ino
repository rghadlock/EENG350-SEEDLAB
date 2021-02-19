#include <Encoder.h>

/*
  MotorSim4.6
  by Nolan Egging (February 2021)
  Colorado School of Mines 
  EENG350 Seed Lab - Mini Project
  Purpose:
  
  Note: 
  There will be a time error after 50 days. This problem is not addressed because the
  program is only for a step input experiment and the program is not meant to run for
  long periods of time.
  Setup:
  
*/

// constants and pins
#define SAMPLE_TIME   20 // 10 ms (100 Hz) sampling time

// global variables
unsigned long currentTime = 0;
int rightA = 2;
int rightB = 6;
int resetButton = 3;
int motorEnable = 4;
int motorDirection = 7;
int motorSpeed = 9;
int counterRight = 0;
double radRightNew = 0;
double radRightOld = 0;
double velRight = 0;
double angVelRight = 0;
int motorFlag =1;
int voltage = 0;
unsigned long lastTime = 0;

Encoder myEnc(rightA, rightB);
// encoder ISR



void resetISR(void){
  myEnc.write(0);
  radRightNew = 0;
  radRightOld = 0;
  angVelRight = 0;
  motorFlag = 0;
}
// setup routine
void setup() {

  // serial communication initialization
  Serial.begin(9600);
  
  //Assign Encoder and button pins as Inputs and motor Pins as output
  pinMode(rightA, INPUT);
  pinMode(rightB, INPUT);
  pinMode(resetButton, INPUT);
  pinMode(motorEnable, OUTPUT);
  digitalWrite(motorEnable, HIGH);
  pinMode(motorDirection, OUTPUT);
  pinMode(motorSpeed, OUTPUT);
  
  

  //Set Up ISR
  
  attachInterrupt(digitalPinToInterrupt(resetButton), resetISR, RISING);

} // end of setup

// loop routine
void loop() {
  
    // measures time for delay
    currentTime = millis();
    if(!motorFlag){
      analogWrite(motorSpeed, 34);
      voltage = 5;
      motorFlag = 1;
    }
    // takes sample
    radRightNew = ((double)myEnc.read()* 6.283)/3200;
    //Serial.println(myEnc.read());
    angVelRight = 1000*(radRightNew - radRightOld)/(SAMPLE_TIME);
    
    //displays sample
    Serial.print(currentTime);
    Serial.print("\t");
    // TODO: motor voltage command printout
    Serial.print(voltage);
    Serial.print("\t");
    // TODO: angular velocity
    Serial.print(angVelRight);
    Serial.print("\n\r");
    radRightOld = radRightNew;
    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));
    
} // end of loop
