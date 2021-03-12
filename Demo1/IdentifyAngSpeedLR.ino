/* IdentifyAngSpeedLR.ino
 * 
 * Purpose : This code is used to determine the transfer function for voltage to 
 * robot angular speed for each wheel.
 * 
 */

// libraries
#include <Encoder.h>

// constants
#define SAMPLE_TIME     20    // sampling time in milliseconds
#define STEP_VOLTAGE    5     // step voltage into the motor
#define MAX_VOLTAGE     7.5   // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.0745   // radius of wheel in meters
#define RAD_IN_DEG      0.01745329  // used for converting degrees to radians.
#define WHEEL_DISTANCE  0.272   // distance between wheels in meters
#define START_DELAY     3000  // start delay in millisconds

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
  digitalWrite(DIRECTION_R, HIGH);
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
    static double angVelocity = 0; // angular velocity in degrees per second
    
    // measures time for delay
    currentTime = millis();

    // starts motor after appropriate time delay
    if (millis() >= START_DELAY) {
      analogWrite(SPEED_R, ((STEP_VOLTAGE * 255) / MAX_VOLTAGE));
      analogWrite(SPEED_L, ((STEP_VOLTAGE * 255) / MAX_VOLTAGE));      
    }
    
    // takes sample of angular velocity
    newDeg_R = ((double)rightEnc.read() * 360) / 3200;
    newDeg_L = ((double)leftEnc.read() * 360) / 3200;

    // calculates angular velocity and angular velocity
    angVel_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
    angVel_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;
    
    // displays samples
    Serial.print((double)currentTime / 1000); // sample time in seconds
    Serial.print("\t");
    Serial.print(angVel_R);
    Serial.print("\t");
    Serial.print(angVel_L);
    Serial.print("\n\r");
    
    // reassigns old degree variables
    oldDeg_R = newDeg_R;
    oldDeg_L = newDeg_L;

    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));
    
} // end of loop
