/* InnerLoop.ino
 * 
 * Purpose : This file is used to implement a pair of controllers for system speed
 * and system angular speed. The speed and angular speed are outputted to the serial
 * monitor.
 * 
 */
 
// important parameters
#define STEP_SPEED_DIS  0.5
#define STEP_SPEED_ROT  0

// libraries
#include <Encoder.h>

// system constants
#define SAMPLE_TIME     40    // sampling time in milliseconds
#define MAX_VOLTAGE     7.5   // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.0745   // radius of wheel in meters
#define WHEEL_DISTANCE  0.290    // distance between wheels in meters
#define START_DELAY     3000  // start delay in millisconds

// conversion constants
#define RAD_IN_DEG      0.01745329
#define METERS_IN_FEET  0.3048 

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

// controller gains
#define KI_DIS          30    // inner loop speed i control
#define KP_DIS          0     // inner loop speed p control
#define KI_ROT          0.15  // inner loop ang speed i control
#define KP_ROT          0.    // inner loop ang speed p control

// sets encoder functions
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB); 

// global variables
double actPos_dis = 0;
double actPos_rot = 0;
double actSpeed_dis = 0;
double actSpeed_rot = 0;
double desPos_dis = 0;
double desPos_rot = 0;
double desSpeed_dis = 0;
double desSpeed_rot = 0;
double errorPos_dis = 0;
double errorPos_rot = 0;
double errorSpeed_dis = 0;
double errorSpeed_rot = 0;
double errorSpeedSum_dis = 0;
double errorSpeedSum_rot = 0;

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

  // writes direction to motor
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, LOW);

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
  static double sumVoltage = 0;
  static double difVoltage = 0;
  static double rightVoltage = 0;
  static double leftVoltage = 0;

  // measures time for delay
  currentTime = millis();

  // starts motor after appropriate time delay
  if (millis() >= START_DELAY) {
    desSpeed_dis = STEP_SPEED_DIS;
    desSpeed_rot = STEP_SPEED_ROT;    
  }

  // takes sample of position
  newDeg_R = ((double)rightEnc.read() * 360) / 3200;
  newDeg_L = -((double)leftEnc.read() * 360) / 3200;

  // calculates positions of system
  actPos_dis = WHEEL_RADIUS * 0.5 * (newDeg_R + newDeg_L) * RAD_IN_DEG;
  actPos_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (newDeg_R - newDeg_L);
  
  // calculates wheel angular speeds
  angVel_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
  angVel_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;

  // calculates speeds of system
  actSpeed_dis = WHEEL_RADIUS * 0.5 * (angVel_R + angVel_L) * RAD_IN_DEG;
  actSpeed_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (angVel_R - angVel_L);

  // control system for sum voltage
  errorSpeed_dis = desSpeed_dis - actSpeed_dis;
  errorSpeedSum_dis += (errorSpeed_dis * SAMPLE_TIME) / 1000;
  sumVoltage = (errorSpeedSum_dis * KI_DIS) + (errorSpeed_dis * KP_DIS);

  // control system for sum voltage
  errorSpeed_dis = desSpeed_dis - actSpeed_dis;
  errorSpeedSum_dis += (errorSpeed_dis * SAMPLE_TIME) / 1000;
  sumVoltage = (errorSpeedSum_dis * KI_DIS) + (errorSpeed_dis * KP_DIS);
  
  // control system for dif voltage
  errorSpeed_rot = desSpeed_rot - actSpeed_rot;
  errorSpeedSum_rot += (errorSpeed_rot * SAMPLE_TIME) / 1000;
  difVoltage = (errorSpeedSum_rot * KI_ROT) + (errorSpeed_rot * KP_ROT);
  
  // determines voltages
  rightVoltage = 0.5 * (sumVoltage + difVoltage);
  leftVoltage = 0.5 * (sumVoltage - difVoltage);
  if (leftVoltage > MAX_VOLTAGE) leftVoltage = MAX_VOLTAGE;
  else if (leftVoltage < -MAX_VOLTAGE) leftVoltage = -MAX_VOLTAGE;
  if (rightVoltage > MAX_VOLTAGE) rightVoltage = MAX_VOLTAGE;
  else if (rightVoltage < -MAX_VOLTAGE) rightVoltage = -MAX_VOLTAGE;

  // determines direction of each motorr and then writes to the motor
  if(rightVoltage < 0) digitalWrite(DIRECTION_R, HIGH);
  else digitalWrite(DIRECTION_R, LOW);
  if(leftVoltage < 0) digitalWrite(DIRECTION_L, HIGH);
  else digitalWrite(DIRECTION_L, LOW); 
  analogWrite(SPEED_R, abs(rightVoltage*255/ MAX_VOLTAGE));
  analogWrite(SPEED_L, abs(leftVoltage*255/ MAX_VOLTAGE));
    
  // displays samples
  Serial.print((double)currentTime / 1000); // sample time in seconds
  Serial.print("\t");
  Serial.print(actSpeed_dis, 2);
  Serial.print("\t");
  Serial.print(actSpeed_rot, 2);
  Serial.print("\t");
  Serial.print(actPos_dis, 2);
  Serial.print("\t");
  Serial.print(actPos_rot, 2);
  Serial.print("\n\r");
  
  // reassigns old degree variables
  oldDeg_R = newDeg_R;
  oldDeg_L = newDeg_L;
  
  // ensures function isn't taking too long
  if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
  
  // creates delay of SAMPLE_TIME ms
  while(millis() < (currentTime + SAMPLE_TIME));
  
} // end of loop
