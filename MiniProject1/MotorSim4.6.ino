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
int counterRight = 0;
double radRight = 0;
double radRightOld = 0;
double velRight = 0;
double angVelRight = 0;
int motorFlag =1;

// encoder ISR
void encoderISR(void){//ISR thats triggered when Pin A changes
  
  if(digitalRead(rightA) != digitalRead(rightB)){//if A and B are different after A changes its moving clockwise
    counterRight++;//cw
  }
  else{
    counterRight--;//ccw
  }

  
} // end of ISR


void resetISR(void){
  radRightNew = 0;
  radRightOld = 0;
  angVelRight = 0;
}
// setup routine
void setup() {

  // serial communication initialization
  Serial.begin(9600);

  //Assign pins as Inputs
  pinMode(rightA, INPUT);
  pinMode(rightB, INPUT);
  pinMode(resetButton, INPUT);

  //Set Up ISR
  attachInterrupt(digitalPinToInterrupt(rightA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(resestButton), resetISR, RISING);

} // end of setup

// loop routine
void loop() {
  
    // measures time for delay
    currentTime = millis();
    if(!motorFlag){
      analogWrite
    }
    // takes sample
    radRightNew = ((double)counterRight * 6.283)/30;
    angVelRight = (radRightNew - radRightOld)/(SAMPLE_RATE/1000);

    //displays sample
    Serial.print(currentTime);
    Serial.print("\t");
    // TODO: motor voltage command printout
    
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
