/* RotationControl.ino
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
#define WHEEL_DISTANCE  0.283
#define RAD_IN_DEG      0.01745329  // used for converting degrees to radians.
#define START_DELAY     3000  // start delay in millisconds
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
double Kv = 0.5;
double KI_L =.03;
double KI_R = .022;
double desAng = 0;
double desAngSpeed_L = 0;
double desAngSpeed_R = 0;
double errAng = 0;
double desRobotAngSpeed = 0;
double desWheelAngSpeed = 0;
double desWheelAngSpeed_R = 0;
double desWheelAngSpeed_L = 0;
double errWheelAngSpeed_R = 0;
double errWheelAngSpeed_L = 0;
double voltage_R = 0;
double voltage_L = 0;
double actWheelAngSpeed_R = 0;
double actWheelAngSpeed_L = 0;
double actAng = 0;
double actRobotAngSpeed = 0;
double errSumAng_R = 0;
double errSumAng_L = 0;

// sets encoder functions
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB);

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
    static double newDeg_R = 0;
    static double newDeg_L = 0;
    static double oldDeg_R = 0;
    static double oldDeg_L = 0;
    static double angVel_R = 0;
    static double angVel_L = 0;

    // measures time for delay
    currentTime = millis();

    // starts motor after appropriate time delay
    if (millis() >= START_DELAY) {
      desAng = 360;     
    }

    // takes angle sample
    newDeg_R = ((double)rightEnc.read() * 360) / 3200;
    newDeg_L = -((double)leftEnc.read() * 360) / 3200;
    actAng = (WHEEL_RADIUS / WHEEL_DISTANCE) * (newDeg_R - newDeg_L);
    errAng = desAng - actAng;

    // calculates angular velocity
    actWheelAngSpeed_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
    actWheelAngSpeed_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;
    
    // calculates desired Robot angular speed and individual wheel speed
    desRobotAngSpeed = errAng * Kv;
    desWheelAngSpeed = (desRobotAngSpeed * WHEEL_DISTANCE)/(2*WHEEL_RADIUS);
    // makes sure the desWheelAngSpeed is below saturation
    if(desWheelAngSpeed > ANG_SATURATION) desWheelAngSpeed = ANG_SATURATION;
    if(desWheelAngSpeed < -ANG_SATURATION) desWheelAngSpeed = -ANG_SATURATION;
    desWheelAngSpeed_R = desWheelAngSpeed;
    desWheelAngSpeed_L = -desWheelAngSpeed;

    // calculates the error sum of angular speeds
    errWheelAngSpeed_R = desWheelAngSpeed_R - actWheelAngSpeed_R;
    errWheelAngSpeed_L = desWheelAngSpeed_L - actWheelAngSpeed_L;
    errSumAng_R = ((errWheelAngSpeed_R * SAMPLE_TIME) / 1000) + errSumAng_R;
    errSumAng_L = ((errWheelAngSpeed_L * SAMPLE_TIME) / 1000) + errSumAng_L;

    // deterimines voltage of the wheels
    voltage_R = errSumAng_R * KI_R;
    voltage_L = errSumAng_L * KI_L;

    // prevents voltage overflow
    if(voltage_R > MAX_VOLTAGE)voltage_R = MAX_VOLTAGE;
    else if(voltage_R < (-1*MAX_VOLTAGE))voltage_R = (-1 *MAX_VOLTAGE);
    if(voltage_L > MAX_VOLTAGE)voltage_L = MAX_VOLTAGE;
    else if(voltage_L < (-1*MAX_VOLTAGE))voltage_L = (-1 *MAX_VOLTAGE);

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
    Serial.print(newDeg_L, 2);
    Serial.print("\t");
    Serial.print("R:");
    Serial.print(newDeg_R, 2);
    Serial.print("\t");
    Serial.print("T:");
    Serial.print(actAng, 2);
    Serial.print("\n\r");
    
    // reassigns old degree variables
    oldDeg_R = newDeg_R;
    oldDeg_L = newDeg_L;
    
    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));

} // end of loop
