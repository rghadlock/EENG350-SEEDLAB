/* RotationControl2.ino
 * 
 * Purpose : This code is used to determine the transfer function for voltage to 
 * robot speed for each wheel.
 * 
 */

 // libraries
#include <Encoder.h>


// constants
#define SAMPLE_TIME     20    // sampling time in milliseconds
#define STEP_VOLTAGE    5     // step voltage into the motor
#define MAX_VOLTAGE     7.5   // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.0745   // radius of wheel in meters
#define WHEEL_DISTANCE  0.300
#define RAD_IN_DEG      0.01745329  // used for converting degrees to radians.
#define START_DELAY     3000  // start delay in millisconds
#define END_DELAY       18000
#define ANG_SATURATION  300

// pins
#define CHANNEL_RA      2     // encoder input pin for right motor channel A
#define CHANNEL_LA      3     // encoder input pin for left motor channel A
#define CHANNEL_RB      5     // encoder input pin for right motor channel B
#define CHANNEL_LB      6     // encoder input pin for left motor channel B
#define ENABLE          4     // enables right motor
#define DIRECTION_R     7     // right motor direction
#define DIRECTION_L     8     // left motor direction
#define SPEED_R         9    // PWM pin for right motor speed
#define SPEED_L         10    // PWM pin for left motor speed   

// global variables
double Kv_L = 0.70;
double Kv_R = 0.70;
double Ki_L = 0.03;
double Ki_R = 0.022;
double desRot = 0;
double desAng_L = 0;
double desAng_R = 0;
double actAng_L = 0;
double actAng_R = 0;
double errorAng_L = 0;
double errorAng_R = 0;
double desAngSpeed_L = 0;
double desAngSpeed_R = 0;
double actAngSpeed_L = 0;
double actAngSpeed_R = 0;
double errorAngSpeed_L = 0;
double errorAngSpeed_R = 0;
double errorAngSpeedSum_L = 0;
double errorAngSpeedSum_R = 0;
double actRot = 0;
double voltage_R = 0;
double voltage_L = 0;

// sets encoder functions
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB);

// function to stop motor
void resetRotation () {
  
  desRot = 0;
  desAng_L = 0;
  desAng_R = 0;
  actAng_L = 0;
  actAng_R = 0;
  errorAng_L = 0;
  errorAng_R = 0;
  desAngSpeed_L = 0;
  desAngSpeed_R = 0;
  actAngSpeed_L = 0;
  actAngSpeed_R = 0;
  errorAngSpeed_L = 0;
  errorAngSpeed_R = 0;
  errorAngSpeedSum_L = 0;
  errorAngSpeedSum_R = 0;
  actRot = 0;
  voltage_R = 0;
  voltage_L = 0;
  
} // end of resetRotation

void setup() {

  // serial communication initialization
  Serial.begin(115200);

  // assigns pins as either inputs or outputs
  pinMode(CHANNEL_RA, INPUT);
  pinMode(CHANNEL_LA, INPUT);
  pinMode(CHANNEL_RB, INPUT);
  pinMode(CHANNEL_LB, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION_R, OUTPUT);
  pinMode(DIRECTION_L, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(SPEED_L, OUTPUT);

  // enables motors
  digitalWrite(ENABLE, HIGH);
  
} // end of setup

void loop() {

    // static variables
    static unsigned long currentTime = 0;
    static double oldDeg_R = 0;
    static double oldDeg_L = 0;
    static double angVel_R = 0;
    static double angVel_L = 0;

    // measures time for delay
    currentTime = millis();

    // starts motor after appropriate time delay
    if (millis() >= START_DELAY) desRot = 360;     

    // stops motor after appropriate time delay
    if ((millis() >= END_DELAY) || (abs(desRot - actRot) < 0.1)) resetRotation();

    // takes angle sample
    actAng_R = ((double)rightEnc.read() * 360) / 3200;
    actAng_L = -((double)leftEnc.read() * 360) / 3200;
    actRot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (actAng_R - actAng_L);

    // calculates angular velocity
    actAngSpeed_R = (1000 * (actAng_R - oldDeg_R)) / SAMPLE_TIME;
    actAngSpeed_L = (1000 * (actAng_L - oldDeg_L)) / SAMPLE_TIME;

    // calculates desired angles
    desAng_R = (WHEEL_DISTANCE * desRot) / (2 * WHEEL_RADIUS);
    desAng_L = -(WHEEL_DISTANCE * desRot) / (2 * WHEEL_RADIUS);
    
    // calculates error in angles
    errorAng_R = desAng_R - actAng_R;
    errorAng_L = desAng_L - actAng_L;

    // determines desired angular speeds
    desAngSpeed_R = Kv_R * errorAng_R;
    desAngSpeed_L = Kv_L * errorAng_L;

    // calculates error and sum of error for angular speeds
    errorAngSpeed_R = desAngSpeed_R - actAngSpeed_R;
    errorAngSpeedSum_R = ((errorAngSpeed_R * SAMPLE_TIME) / 1000) + errorAngSpeedSum_R;
    errorAngSpeed_L = desAngSpeed_L - actAngSpeed_L;
    errorAngSpeedSum_L = ((errorAngSpeed_L * SAMPLE_TIME) / 1000) + errorAngSpeedSum_L;

    // determines voltage of motors and prevents overflow
    voltage_R = errorAngSpeedSum_R * Ki_R;
    if(voltage_R > MAX_VOLTAGE) voltage_R = MAX_VOLTAGE;
    else if(voltage_R < (-1*MAX_VOLTAGE)) voltage_R = (-1 *MAX_VOLTAGE);
    voltage_L = errorAngSpeedSum_L * Ki_L;
    if(voltage_L > MAX_VOLTAGE) voltage_L = MAX_VOLTAGE;
    else if(voltage_L < (-1*MAX_VOLTAGE)) voltage_L = (-1 *MAX_VOLTAGE);

    // determines direction of each motorr and then writes to the motor
    if(voltage_R < 0) digitalWrite(DIRECTION_R, HIGH);
    else digitalWrite(DIRECTION_R, LOW);
    if(voltage_L < 0) digitalWrite(DIRECTION_L, HIGH);
    else digitalWrite(DIRECTION_L, LOW); 
    analogWrite(SPEED_R, abs(voltage_R*255/ MAX_VOLTAGE));
    analogWrite(SPEED_L, abs(voltage_L*255/ MAX_VOLTAGE));

    // displays samples
    Serial.print((double)currentTime / 1000); // sample time in seconds
    Serial.print("\t");
    Serial.print("L:");
    Serial.print(actAng_L, 2);
    Serial.print("\t");
    Serial.print("R:");
    Serial.print(actAng_R, 2);
    Serial.print("\t");
    Serial.print("T:");
    Serial.print(actRot, 2);
    Serial.print("\n\r");

    // reassigns old degree variables
    oldDeg_R = actAng_R;
    oldDeg_L = actAng_L;
    
    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));

} // end of loop
