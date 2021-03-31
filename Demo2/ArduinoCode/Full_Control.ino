/* Full_Control.ino
 * 
 * Purpose : The files implements the full control system for the robot. Using serial communication,
 * various position, rotation, and speed functions can be called.
 * 
 */

// important parameters
#define MAX_VOLTAGE       8.2       // maximum voltage of the input into the motor
#define CIRCLE_RADIUS     0.4       // radius of circle robot will drive
#define CIRCLE_TIME       4.0       // time for robot to drive circle
#define CIRCLE_THRESH_DIS 0.05      // threshhold to stop circle movement
#define CIRCLE_THRESH_ROT 1.00      // threshhold to stop circle movement
#define CIRCLE            2.5233    // circle circumfrence

// libraries
#include <Encoder.h>

// system constants
#define SAMPLE_TIME     30.0     // sampling time in milliseconds
#define WHEEL_RADIUS    0.07485   // radius of wheel in meters
#define WHEEL_DISTANCE  0.29750    // distance between wheels in meters

// conversion constants
#define RAD_IN_DEG      0.01745329
#define METERS_IN_FEET  0.3048 

// inner-loops controller gains and constants
#define KP_DIS          0.00
#define KI_DIS          25.0
#define KP_ROT          0.00
#define KI_ROT          0.20 

// outer-loops controller gains and constants
#define KPO_DIS         2.00
#define KDO_DIS         0.88
#define SPEED_SAT_DIS   0.75
#define KPO_ROT         1.50
#define KDO_ROT         0.00
#define SPEED_SAT_ROT   90.0

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

// sets encoder variables
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB); 

// global variables for control system
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

// global variabls for system mode
bool control[5] = {false, false, false, false, false}; // [distance, rotation, speed, angular speed, circle]


// search - rotates the robot at a constant rate until interrupted
// input is in (deg/s)*100
// Ex: input of 36000 means 360 deg/sec)
void search(long searchSpeed) {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = false;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desSpeed_rot = ((double)searchSpeed) / 100;
}


// aim - fine tunes the angle of the robot to match the marker
// input is in deg*100
// Ex: input of 10 means 0.1 degrees
void aim(long aimAng) {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_rot = ((double)aimAng) / 100;
}

// drive - drives the robot a specific distance
// input is in millimeters
void drive (long driveDis) {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_dis = actPos_dis + (((double)driveDis) / 1000);
}


// rotate - rotates the robot a predifened angle
// no input
void rotate() {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_rot = actPos_rot - 90;
}


// circle - drives the robot in a predifined circle
// no input
void circle() {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = false;
  control[1] = false;
  control[2] = true;
  control[3] = true;
  control[4] = true;
  desSpeed_dis = 3.14159265 * (((double)CIRCLE_RADIUS) / ((double)CIRCLE_TIME));
  desSpeed_rot = 180.0 / ((double)CIRCLE_TIME);
}

// correct - corrects robot position to past desired distance and rotation
// no input
void correct() {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_dis += CIRCLE;
  desPos_rot += 360;
}

// kill - stops robot from doing anything else
// no input
void kill() {
  control[0] = false;
  control[1] = false;
  control[2] = false;
  control[3] = false;
  control[4] = false;
  digitalWrite(ENABLE, LOW);
}


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

  // !!!!!!!!! simulation test area !!!!!!!!!
  static bool stage1_f = false;
  static bool stage2_f = false;
  static bool stage3_f = false;
  static bool stage4_f = false;
  static bool stage5_f = false;
  static bool stage6_f = false;
  if ((millis() >= 2000) && (!stage1_f)) {
    circle();
    stage1_f = true;
  }
  if ((millis() >= 20000) && (!stage2_f)) {
    kill();
    stage2_f = true;
  }
  // !!!!!!!!! end simulation test area !!!!!

  // takes samples of system
  newDeg_R = ((double)rightEnc.read() * 360) / 3200;
  newDeg_L = -((double)leftEnc.read() * 360) / 3200;
  actPos_dis = WHEEL_RADIUS * 0.5 * (newDeg_R + newDeg_L) * RAD_IN_DEG;
  actPos_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (newDeg_R - newDeg_L);
  angVel_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
  angVel_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;
  actSpeed_dis = WHEEL_RADIUS * 0.5 * (angVel_R + angVel_L) * RAD_IN_DEG;
  actSpeed_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (angVel_R - angVel_L);

  // distance outer-loop control
  if (control[0]) {
    errorPos_dis = desPos_dis - actPos_dis;
    errorPosChange_dis = ((errorPos_dis - errorPosOld_dis) * 1000.0) / SAMPLE_TIME;
    errorPosOld_dis = errorPos_dis;
    desSpeed_dis = (errorPos_dis * KPO_DIS) + (errorPosChange_dis * KDO_DIS);
    if (desSpeed_dis > SPEED_SAT_DIS) desSpeed_dis = SPEED_SAT_DIS;
    else if (desSpeed_dis < -SPEED_SAT_DIS) desSpeed_dis = -SPEED_SAT_DIS;
  } 
  
  // rotation outer-loop control
  if (control[1]) {
    errorPos_rot = desPos_rot - actPos_rot;
    errorPosChange_rot = ((errorPos_rot - errorPosOld_rot) * 1000.0) / SAMPLE_TIME;
    errorPosOld_rot = errorPos_dis;
    desSpeed_rot = (errorPos_rot * KPO_ROT) + (errorPosChange_rot * KDO_ROT);
    if (desSpeed_rot > SPEED_SAT_ROT) desSpeed_rot = SPEED_SAT_ROT;
    else if (desSpeed_rot < -SPEED_SAT_ROT) desSpeed_rot = -SPEED_SAT_ROT; 
  }

  // distance inner-loop control
  if (control[2]) {
    errorSpeed_dis = desSpeed_dis - actSpeed_dis;
    errorSpeedSum_dis += (errorSpeed_dis * SAMPLE_TIME) / 1000.0;
    sumVoltage = (errorSpeedSum_dis * KI_DIS) + (errorSpeed_dis * KP_DIS);
  }

  // rotation inner-loop control
  if (control[3]) {
    errorSpeed_rot = desSpeed_rot - actSpeed_rot;
    errorSpeedSum_rot += (errorSpeed_rot * SAMPLE_TIME) / 1000.0;
    difVoltage = (errorSpeedSum_rot * KI_ROT) + (errorSpeed_rot * KP_ROT);
  }

  // circle control
  if (control[4]) {
    if (abs(actPos_dis - (desPos_dis + CIRCLE)) <= CIRCLE_THRESH_DIS) {
      if (abs(actPos_rot - (desPos_rot + 360.0)) <= CIRCLE_THRESH_ROT){
        correct();
      }
    }
  }

  // determines individual voltages
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

  // displays samples for debugging purposes
  Serial.print((double)currentTime / 1000); // sample time in seconds
  Serial.print("\t");
  Serial.print(actSpeed_dis, 2);
  Serial.print("\t");
  Serial.print(actSpeed_rot, 2);
  Serial.print("\t\t");
  Serial.print(actPos_dis, 4);
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
