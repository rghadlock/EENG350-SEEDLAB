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

// pin names

// constants
#define SAMPLE_TIME   10 // 10 ms (100 Hz) sampling time

// global variables
unsigned long currentTime = 0;

// setup routine
void setup() {

  // serial communication initialization
  Serial.begin(9600);

} // end of setup

// loop routine
void loop() {

    // measures time for delay
    currentTime = millis();

    // takes sample
    // TODO

    //displays sample
    Serial.print(currentTime);
    Serial.print("\t");
    // TODO: motor voltage command printout
    Serial.print("\t");
    // TODO: angular velocity
    Serial.print("\n\r");

    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));

} // end of loop
