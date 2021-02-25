/*
  Motor Controller
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
#define MAX_VOLTAGE     8.2   // maximum voltage of the input into the motor
#define SLAVE_ADDRESS   0x04

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
double posNow = 0;        // holds new radian reading
double oldRadians = 0;        // holds old raidan reading
double angVelocity = 0;       // holds calculated angular velocity
int voltage = 0;              // holds equivalent voltage put into the motor
byte outVal[2] = {0};         // holds bytes to send to pi
byte dataRec[4] = {0};
int outPos = 0;               // hold rotary encoder val

double posDes = 0;
double posErr = 0;
double inputVoltage = 0;
double posErrSum = 0;
//double kp = 2.10;
//double ki = 0.137;
double kp = 3.3;
double ki = 0.2291;
int posRec = 0;

// sets encoder function
Encoder motorEnc(CHANNEL_A, CHANNEL_B);


// ISR that detects when the reset button is pressed
void resetISR(void){

   // changes desired button
  if (motorOnFlag == 0) {
    motorOnFlag = 1;
    posDes = 6.28;
  } else {
    motorOnFlag  = 0;
    posDes = 0;
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
    
    // takes position sample
    posNow = ((double)motorEnc.read() * 6.283) / 3200;
    outPos = motorEnc.read();                             //?????
  //after desired position is recieved from pi, change into radians. 
    
    posDes = (double)posRec / (double) 1000;
    
    // controller implementation
    posErr = posDes - posNow;
    posErrSum = ((posErr * SAMPLE_TIME) / 1000) + posErrSum; //add on error before for integral component
    inputVoltage = (posErr * kp) + ((posErrSum * ki * SAMPLE_TIME) / 1000); //apply controller values to convert rad to voltage 
    if( inputVoltage > MAX_VOLTAGE){
      inputVoltage = MAX_VOLTAGE;
    }
    else if(inputVoltage < (-1* MAX_VOLTAGE)){
      inputVoltage = (-1 * MAX_VOLTAGE);
    }
    if(inputVoltage < 0){
      digitalWrite(MOTOR_DIRECTION, LOW);
    }
    else{
      digitalWrite(MOTOR_DIRECTION, HIGH);
    }
    analogWrite(MOTOR_SPEED, (abs(inputVoltage * 255) / MAX_VOLTAGE));
    
    //displays data
    Serial.print((double)currentTime / 1000); // sample time in seconds
    Serial.print("\t");
    Serial.print(inputVoltage);
    Serial.print("\t");
    Serial.print(posNow);
    Serial.print("\t");
    Serial.print(posErr);
    Serial.print("\t");
    Serial.print(posErrSum);
    Serial.print("\t");
    Serial.print(MAX_VOLTAGE);
    Serial.print("\n\r");
   
    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));
    
} // end of loop

void receiveData(int byteCount) {
  int k = 0;
  while (Wire.available()) {
    dataRec[k] = Wire.read();
   
    k++;
  }
  posRec = (dataRec[1] << 8) | (dataRec[2]);
  
}
void sendData(){
 //shift bits to get specific bytes from outPos
  outVal[0] = outPos >> 8;
  outVal[1] = outPos & 0x00FF;
  Serial.println();
  Wire.write(outVal, 2);
  
}
