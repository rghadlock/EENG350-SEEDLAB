/* DistanceIL_code.ino
 * 
 * Purpose : Inner loop control for distance
 * 
 */

// important parameters
#define STEP_SPEED          0.5

// libraries
#include <Encoder.h>

// system constants
#define SAMPLE_TIME     40    // sampling time in milliseconds
#define MAX_VOLTAGE     7.5   // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.0745   // radius of wheel in meters
#define START_DELAY     3000  // start delay in millisconds

// conversion constants
#define RAD_IN_DEG      0.01745329  // used for converting degrees to radians.

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


// sets encoder functions
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB); 

// full system global variables
double voltage_L = 0;
double voltage_R = 0;

// innerloop global variables
double Ki_L = 10.5;
double Kp_L = 0;
double Ki_R = 11;
double Kp_R = 0;
double desSpeed_L = 0;
double desSpeed_R = 0;
double actSpeed_L = 0;
double actSpeed_R = 0;
double errorSpeed_L = 0;
double errorSpeed_R = 0;
double errorSpeedSum_L = 0;
double errorSpeedSum_R = 0;

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
  
  // measures time for delay
  currentTime = millis();

  // starts motor after appropriate time delay
  if (millis() >= START_DELAY) {
    desSpeed_L = STEP_SPEED;
    desSpeed_R = STEP_SPEED;    
  }

  // takes sample of angular velocity
  newDeg_R = ((double)rightEnc.read() * 360) / 3200;
  newDeg_L = ((double)leftEnc.read() * 360) / 3200;

  // calculates angular velocity and straightforward velocity
  angVel_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
  angVel_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;
  actSpeed_R = WHEEL_RADIUS * angVel_R * RAD_IN_DEG;
  actSpeed_L = -WHEEL_RADIUS * angVel_L * RAD_IN_DEG;

  // determines error and error sum for inner loop
  errorSpeed_L = desSpeed_L - actSpeed_L;
  errorSpeed_R = desSpeed_R - actSpeed_R;
  errorSpeedSum_L += (errorSpeed_L * SAMPLE_TIME) / 1000;
  errorSpeedSum_R += (errorSpeed_R * SAMPLE_TIME) / 1000;

  // determines motor voltages
  voltage_L = (errorSpeedSum_L * Ki_L) + (errorSpeed_L * Kp_L);
  voltage_R = (errorSpeedSum_R * Ki_R) + (errorSpeed_R * Kp_R);
  if (voltage_L > MAX_VOLTAGE) voltage_L = MAX_VOLTAGE;
  else if (voltage_L < -MAX_VOLTAGE) voltage_L = -MAX_VOLTAGE;
  if (voltage_R > MAX_VOLTAGE) voltage_R = MAX_VOLTAGE;
  else if (voltage_R < -MAX_VOLTAGE) voltage_R = -MAX_VOLTAGE;

  // writes motor voltage
  analogWrite(SPEED_L, ((voltage_L * 255) / MAX_VOLTAGE));   
  analogWrite(SPEED_R, ((voltage_R * 255) / MAX_VOLTAGE)); 
  
  // displays samples
  Serial.print((double)currentTime / 1000); // sample time in seconds
  Serial.print("\t");
  Serial.print(actSpeed_L, 2);
  Serial.print("\t");
  Serial.print(actSpeed_R, 2);
  Serial.print("\n\r");
  
  // reassigns old degree variables
  oldDeg_R = newDeg_R;
  oldDeg_L = newDeg_L;
  
  // ensures function isn't taking too long
  if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
  
  // creates delay of SAMPLE_TIME ms
  while(millis() < (currentTime + SAMPLE_TIME));
  
} // end of loop
