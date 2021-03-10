/* closedLoopControl.ino
 * 
 * Purpose : This code is used to determine the transfer function for voltage to 
 * robot speed for each wheel.
 * 
 */

 // libraries
#include <Encoder.h>

// constants
#define SAMPLE_TIME     10    // sampling time in milliseconds
#define STEP_VOLTAGE    5     // step voltage into the motor
#define MAX_VOLTAGE     7.5   // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.075   // radius of wheel in meters
#define RAD_IN_DEG      0.01745329  // used for converting degrees to radians.
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

double KV_L = 0.4;
double KV_R = 0.4;
double KI_L = 10;
double KI_R = 10.5;
double desSpeed_L = 0;
double desSpeed_R = 0;
double errPos_L = 0;
double errPos_R = 0;
double errSpeed_R = 0;
double errSpeed_L = 0;
double voltage_R = 0;
double voltage_L = 0;
double actSpeed_R = 0;
double actSpeed_L = 0;
double actPos_R = 0;
double actPos_L = 0;
double desPos = 0;
double errSum_R = 0;
double errSum_L = 0;

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
      desPos = 1;     
    }
    
    // takes sample of angular velocity
    newDeg_R = ((double)rightEnc.read() * 360) / 3200;
    newDeg_L = ((double)leftEnc.read() * 360) / 3200;

    // calculates angular velocity and straightforward velocity
    angVel_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
    angVel_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;
    actSpeed_R = WHEEL_RADIUS * angVel_R * RAD_IN_DEG;
    actSpeed_L = -WHEEL_RADIUS * angVel_L * RAD_IN_DEG;

    actPos_R = (((double)rightEnc.read() * 6.2832) / 3200)* WHEEL_RADIUS;
    actPos_L = (-1*(((double)leftEnc.read() * 6.2832) / 3200)* WHEEL_RADIUS);

    errPos_R = desPos - actPos_R;
    errPos_L = desPos - actPos_L;

    desSpeed_R = errPos_R* KV_R;
    desSpeed_L = errPos_L* KV_L;

    errSpeed_R = desSpeed_R - actSpeed_R;
    errSpeed_L = desSpeed_L - actSpeed_L;

    errSum_R = ((errSpeed_R * SAMPLE_TIME) / 1000) + errSum_R;
    errSum_L = ((errSpeed_L * SAMPLE_TIME) / 1000) + errSum_L;
  
    voltage_R = errSum_R * KI_R;
    voltage_L = errSum_L * KI_L;

    if(voltage_R > MAX_VOLTAGE)voltage_R = MAX_VOLTAGE;
    else if(voltage_R < (-1*MAX_VOLTAGE))voltage_R = (-1 *MAX_VOLTAGE);
    if(voltage_L > MAX_VOLTAGE)voltage_L = MAX_VOLTAGE;
    else if(voltage_L < (-1*MAX_VOLTAGE))voltage_L = (-1 *MAX_VOLTAGE);
    if(voltage_R < 0)digitalWrite(DIRECTION_R, HIGH);
    else digitalWrite(DIRECTION_R, LOW);
    if(voltage_L < 0)digitalWrite(DIRECTION_L, HIGH);
    else digitalWrite(DIRECTION_L, LOW);  
    
    analogWrite(SPEED_R, abs(voltage_R*255/ MAX_VOLTAGE));
    analogWrite(SPEED_L, abs(voltage_L*255/ MAX_VOLTAGE));
    
    // displays samples
    Serial.print((double)currentTime / 1000); // sample time in seconds
    Serial.print("\t");
    Serial.print(actPos_R);
    Serial.print("\t");
    Serial.print(actPos_L);
    Serial.print("\t");
    Serial.print(actSpeed_R);
    Serial.print("\t");
    Serial.print(actSpeed_L);
    Serial.print("\n\r");
    
    // reassigns old degree variables
    oldDeg_R = newDeg_R;
    oldDeg_L = newDeg_L;

    // ensures function isn't taking too long
    if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
    
    // creates delay of SAMPLE_TIME ms
    while(millis() < (currentTime + SAMPLE_TIME));
    
} // end of loop
