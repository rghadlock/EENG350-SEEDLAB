/*
  MotorSim4.6
  by Jacob Bommersbach Nolan Egging (February 2021)
  Colorado School of Mines 
  EENG350 Seed Lab - Mini Project

  Purpose: 
  
  Note: 
  There will be a time error after 50 days. This problem is not addressed because the
  program is only for a step input experiment and the program is not meant to run for
  long periods of time.
  Setup:
  
*/

// libraries
#include <Encoder.h>
#include <Wire.h>

// constants
#define SAMPLE_TIME     10    // sampling time
#define STEP_VOLTAGE    1     // equivalent input voltage into the motor
#define MAX_VOLTAGE     7.2   // maximum voltage of the input into the motor
#define SLAVE_ADDRESS 0x04

// pins
#define CHANNEL_A       2     // encoder input pin for channel A  
#define RESET_BUTTON    3     // input pin for active high reset button
#define MOTOR_ENABLE    4     // output pin that needs to be high for the motor to be enabled
#define CHANNEL_B       6     // encoder input pin for channel B
#define MOTOR_DIRECTION 7     // output pin for motor direction
#define MOTOR_SPEED     9     // output pin for PWM that controls motor speed

// global variables
unsigned long currentTime = 0;// holds time in ms of the start of the loop routine
int motorOnFlag = 0;          // flag that represents whether or not the motor is on or off
double newRadians = 0;        // holds new radian reading
double oldRadians = 0;        // holds old raidan reading
double angVelocity = 0;       // holds calculated angular velocity
int voltage = 0;              // holds equivalent voltage put into the motor
byte outVal[2] = {0};         // holds bytes to send to pi
int outPos = 0;               // hold rotary encoder val

// sets encoder function
Encoder motorEnc(CHANNEL_A, CHANNEL_B);


// ISR that detects when the reset button is pressed
void resetISR(void){

   // turns on or off motor depending on current state
  if (motorOnFlag == 0) {

    // turns on motor
    analogWrite(MOTOR_SPEED, ((STEP_VOLTAGE * 255) / MAX_VOLTAGE));
    motorOnFlag = 1;
    voltage = STEP_VOLTAGE;
    
  } else {

    // turns off motor
    motorEnc.write(0);
    newRadians = 0;
    oldRadians = 0;
    angVelocity = 0;
    analogWrite(MOTOR_SPEED, 0);
    motorOnFlag  = 0;
    voltage = 0;
    
  } // end of (motorOnFlag == 0) if else branch
  
} // end of ISR


// setup routine
void setup() {

  // serial communication initialization
  Serial.begin(115200);
  
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  // assign Encoder and button pins as Inputs and motor Pins as output
  pinMode(CHANNEL_A, INPUT);
  pinMode(CHANNEL_B, INPUT);
  pinMode(RESET_BUTTON, INPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_DIRECTION, OUTPUT);
  pinMode(MOTOR_SPEED, OUTPUT);

  // enables motor
  digitalWrite(MOTOR_DIRECTION, HIGH);
  digitalWrite(MOTOR_ENABLE, HIGH);
  
  // set Up ISR
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON), resetISR, RISING);

} // end of setup


// loop routine
void loop() {
  
    // measures time for delay
    currentTime = millis();
    
    // takes sample and calculates angular velocity
    newRadians = ((double)motorEnc.read() * 6.283) / 3200;
    outPos = motorEnc.read();
    angVelocity = (1000 * (newRadians - oldRadians)) / SAMPLE_TIME;
    
    //displays sample
    Serial.print((double)currentTime / 1000); // sample time in seconds
    Serial.print("\t");
    Serial.print(voltage);
    Serial.print("\t");
    Serial.print(angVelocity);
    Serial.print("\n\r");

    // resets old radians variable
    oldRadians = newRadians;
    
    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));
    
} // end of loop

void receiveData(int byteCount) {

}
void sendData(){
  //shift bits to get specific bytes from sensorVal
  outVal[0] = outPos >> 8;
  outVal[1] = outPos & 0x00FF;
  Serial.println();
  Wire.write(outVal, 2);
  
}
