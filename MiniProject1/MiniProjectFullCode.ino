/*
  Full Mini Project Arduino Code
  by Group 9: Harrison Baer, Jacob Bommersbach, Nolan Egging, Robert Hadlokc (February 2021)
  Colorado School of Mines 
  EENG350 Seed Lab - Mini Project
  
  Purpose: This script implements the motor controller. Desired position is sent in via I2C and a corresponding voltage is
  sent to the motor.
  
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
double posNow = 0;            // holds new radian reading
double oldRadians = 0;        // holds old raidan reading
double angVelocity = 0;       // holds calculated angular velocity
int voltage = 0;              // holds equivalent voltage put into the motor
byte outVal[2] = {0};         // holds bytes to send to pi
byte dataRec[4] = {0};
int outPos = 0;               // hold rotary encoder val in counts

double posDes = 0;            //Desired Position variable in rad
double posErr = 0;            //Error Position variable in rad
double inputVoltage = 0;      //Voltage sent to Motor variable in Volts
double posErrSum = 0;         //sum of position error variable in rad*s
double kp = 3.3;              //Proportional Gain value in Volts/Rad
double ki = 0.2291;           //Integral Gain value in Volts/(Rad*s)
int posRec = 0;               //Desired Position from I2C in int form 

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
  
   
    currentTime = millis();                                   //Measures time for delay
   
    posNow = ((double)motorEnc.read() * 6.283) / 3200;        //Convert counts to angular position in rad
    outPos = motorEnc.read();                                 //Read encoder count to send to Pi                   
    
    posDes = (double)posRec / (double) 1000;                  //Convert the Desired Position from Pi to a radian value
    
    // controller implementation
    posErr = posDes - posNow;                                 //Calculate Error Position
    posErrSum = ((posErr * SAMPLE_TIME) / 1000) + posErrSum;  //Add on error before for integral component
    inputVoltage = (posErr * kp) + (posErrSum * ki );         //Apply controller values to convert rad to voltage 
    
    if( inputVoltage > MAX_VOLTAGE){                          //Check if the input voltage will be saturated
      inputVoltage = MAX_VOLTAGE;                             //Set voltage to max
    }
    else if(inputVoltage < (-1* MAX_VOLTAGE)){                //Check if the input voltage will be saturated on the negative side
      inputVoltage = (-1 * MAX_VOLTAGE);                      //Set voltage to Max in oppisite direction
    }
    if(inputVoltage < 0){                                     //Check if Input voltage is negative
      digitalWrite(MOTOR_DIRECTION, LOW);                     //Set the Motor to turn Counter-Clock-Wise
    }
    else{                                                     //Otherwise
      digitalWrite(MOTOR_DIRECTION, HIGH);                    //Set Motor to turn Clock-Wise
    }
    analogWrite(MOTOR_SPEED, (abs(inputVoltage * 255) / MAX_VOLTAGE)); // Write the voltage to the Motor
    
    //Displays in the serial monitor the time, input voltage, current Radial position, error position, and sum of position error seperated by tabs for monitoring purposes

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
