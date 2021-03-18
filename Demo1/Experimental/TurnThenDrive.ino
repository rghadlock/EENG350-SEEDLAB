/* TurnThenDrive.ino
 * 
 * Purpose : This code is used to determine the transfer function for voltage to 
 * robot speed for each wheel.
 * 
 */

// important parameters (note: distance is in feet)
#define DISTANCE          1
#define ROTATION          -180

// libraries
#include <Encoder.h>

// system constants
#define SAMPLE_TIME     30.0     // sampling time in milliseconds
#define MAX_VOLTAGE     8.1      // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.07485   // radius of wheel in meters
#define WHEEL_DISTANCE  0.29750    // distance between wheels in meters

// delay times in milliseconds
#define START_DELAY     1500 
#define ROTATE_DELAY    16500
#define DRIVE_DELAY     24000

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

// innner-loop controller gains
#define KI_DIS          30   
#define KP_DIS          0     
#define KI_ROT          0.20
#define KP_ROT          0    

// outer-loop controller gains and variables    
#define KDO_DIS_1           0.24
#define KPO_DIS_1           1.0 
#define KDO_ROT_1           0.1
#define KPO_ROT_1           4.0
#define SPEED_SAT_DIS_1     0.5
#define SPEED_SAT_ROT_1     45
#define KDO_DIS_2           0.24
#define KPO_DIS_2           1.0 
#define KDO_ROT_2           0.0
#define KPO_ROT_2           4.0
#define SPEED_SAT_DIS_2     0.5
#define SPEED_SAT_ROT_2     180

// controller globals
double KDO_DIS = 0;
double KPO_DIS = 0;
double KDO_ROT = 0;
double KPO_ROT = 0;
double SPEED_SAT_DIS = 0;
double SPEED_SAT_ROT = 0;

// sets encoder variables
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB); 

// global variables
double actPos_dis = 0;
double actPos_rot = 0;
double desPos_dis = 0;
double desPos_rot = 0;
double errorPos_dis = 0;
double errorPos_rot = 0;
double actSpeed_dis = 0;
double actSpeed_rot = 0;
double desSpeed_dis = 0;
double desSpeed_rot = 0;
double errorSpeed_dis = 0;
double errorSpeed_rot = 0;
double errorSpeedSum_dis = 0;
double errorSpeedSum_rot = 0;
double errorPosOld_dis = 0;
double errorPosOld_rot = 0;
double errorPosChange_dis = 0;
double errorPosChange_rot = 0;

// global flags
bool start_f = 0;
bool rotate_f = 0;
bool drive_f = 0;

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

  // rotates system after appropriate time delay
  if ((millis() >= START_DELAY) && (!start_f)) {
    KDO_DIS = KDO_DIS_1;
    KPO_DIS = KPO_DIS_1;
    KDO_ROT = KDO_ROT_1;
    KPO_ROT = KPO_ROT_1;
    SPEED_SAT_DIS = SPEED_SAT_DIS_1;
    SPEED_SAT_ROT = SPEED_SAT_ROT_1;
    desPos_dis = 0;
    desPos_rot = ROTATION;    
    start_f = 1;
  }

  // drives system after appropriate time delay
  if ((millis() >= ROTATE_DELAY) && (start_f) && (!rotate_f)) {
    errorSpeedSum_dis = 0;
    errorSpeedSum_rot = 0;
    KDO_DIS = KDO_DIS_2;
    KPO_DIS = KPO_DIS_2;
    KDO_ROT = KDO_ROT_2;
    KPO_ROT = KPO_ROT_2;
    SPEED_SAT_DIS = SPEED_SAT_DIS_2;
    SPEED_SAT_ROT = SPEED_SAT_ROT_2;
    desPos_dis = DISTANCE * METERS_IN_FEET;
    desPos_rot = ROTATION;    
    rotate_f = 1;
  }

  // stops motor after appropriate time delay
  if ((millis() >= DRIVE_DELAY) && (rotate_f) && (!drive_f)) {
    digitalWrite(ENABLE, LOW);
    drive_f = 1;
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

  // outer loop control system for distance
  errorPos_dis = desPos_dis - actPos_dis;
  errorPosChange_dis = ((errorPos_dis - errorPosOld_dis) * 1000.0) / SAMPLE_TIME;
  errorPosOld_dis = errorPos_dis;
  desSpeed_dis = (errorPos_dis * KPO_DIS) + (errorPosChange_dis * KDO_DIS);
  if (desSpeed_dis > SPEED_SAT_DIS) desSpeed_dis = SPEED_SAT_DIS;
  else if (desSpeed_dis < -SPEED_SAT_DIS) desSpeed_dis = -SPEED_SAT_DIS;

  // outer loop control system for rotation
  errorPos_rot = desPos_rot - actPos_rot;
  errorPosChange_rot = ((errorPos_rot - errorPosOld_rot) * 1000.0) / SAMPLE_TIME;
  errorPosOld_rot = errorPos_dis;
  desSpeed_rot = (errorPos_rot * KPO_ROT) + (errorPosChange_rot * KDO_ROT);
  if (desSpeed_rot > SPEED_SAT_ROT) desSpeed_rot = SPEED_SAT_ROT;
  else if (desSpeed_rot < -SPEED_SAT_ROT) desSpeed_rot = -SPEED_SAT_ROT;
  
  // control system for sum voltage
  errorSpeed_dis = desSpeed_dis - actSpeed_dis;
  errorSpeedSum_dis += (errorSpeed_dis * SAMPLE_TIME) / 1000.0;
  sumVoltage = (errorSpeedSum_dis * KI_DIS) + (errorSpeed_dis * KP_DIS);
  
  // control system for dif voltage
  errorSpeed_rot = desSpeed_rot - actSpeed_rot;
  errorSpeedSum_rot += (errorSpeed_rot * SAMPLE_TIME) / 1000.0;
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
  Serial.print("\t\t");
  Serial.print(actPos_dis / METERS_IN_FEET, 4);
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
