/* NewEncoders.ino
 * 
 * Purpose : 
 * 
 */

// important parameters (note: distance is in feet)
#define STEP_SUM_VOLTAGE  10
#define STEP_DIF_VOLTAGE  0

// system constants
#define SAMPLE_TIME     30.0     // sampling time in milliseconds
#define MAX_VOLTAGE     8.2      // maximum voltage of the input into the motor
#define WHEEL_RADIUS    0.07485   // radius of wheel in meters
#define WHEEL_DISTANCE  0.29750    // distance between wheels in meters

// delay times in milliseconds
#define START_DELAY     3000 
#define END_DELAY       10000

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

// global variables
double actPos_dis = 0;
double actPos_rot = 0;
double actSpeed_dis = 0;
double actSpeed_rot = 0;
int counter_R = 0;
int counter_L = 0;
unsigned long lastTime_R = 0; 
unsigned long newTime_R = 0; 
int timeDiff_R = 0;
unsigned long lastTime_L = 0; 
unsigned long newTime_L = 0; 
int timeDiff_L = 0;
double newDeg_R = 0;
double newDeg_L = 0;
double oldDeg_R = 0;
double oldDeg_L = 0;
double angVel_R = 0;
double angVel_L = 0;

// global flags
bool start_f = 0;
bool end_f = 0;

void rightISR(void){//ISR thats triggered when right Pin A changes
  
  if(digitalRead(CHANNEL_RA) != digitalRead(CHANNEL_RB)){//if A and B are different after A changes its moving clockwise
    counter_R++;//cw
  }
  else{
    counter_R--;//ccw
  }
  newTime_R = micros();
  newDeg_R = (counter_R * 360) / 3200;
  timeDiff_R = newTime_R - lastTime_R;
  angVel_R = (1000000 * (newDeg_R - oldDeg_R)) / timeDiff_R;
  actPos_dis = WHEEL_RADIUS * 0.5 * (newDeg_R + newDeg_L) * RAD_IN_DEG;
  actPos_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (newDeg_R - newDeg_L);
  lastTime_R = newTime_R;
  oldDeg_R = newDeg_R;
}

void leftISR(void){//ISR thats triggered when left Pin A changes

  if(digitalRead(CHANNEL_LA) != digitalRead(CHANNEL_LB)){//if A and B are different after A changes its moving clockwise
    counter_L++;//cw
  }
  else{
    counter_L--;//ccw
  }
  newTime_L = micros();
  newDeg_L = (counter_L * 360) / 3200;
  timeDiff_L = newTime_L - lastTime_L;
  angVel_L = (1000000 * (newDeg_L - oldDeg_L)) / timeDiff_L;
  actPos_dis = WHEEL_RADIUS * 0.5 * (newDeg_R + newDeg_L) * RAD_IN_DEG;
  actPos_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (newDeg_R - newDeg_L);
  lastTime_L = newTime_L;
  oldDeg_L = newDeg_L;

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
  attachInterrupt(digitalPinToInterrupt(CHANNEL_RA), rightISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_LA), leftISR, RISING);

} // end of setup

void loop() {

  // static variables
  static unsigned long currentTime = 0;
  static double sumVoltage = 0;
  static double difVoltage = 0;
  static double rightVoltage = 0;
  static double leftVoltage = 0;

  // measures time for delay
  currentTime = millis();

  // starts motor after appropriate time delay
  if ((millis() >= START_DELAY) && (!start_f)) {
    sumVoltage = STEP_SUM_VOLTAGE;
    difVoltage = STEP_DIF_VOLTAGE;   
    start_f = 1;
  }

  // stops motor after appropriate time delay
  if ((millis() >= END_DELAY) && (start_f) && (!end_f)) {
    sumVoltage = 0;
    difVoltage = 0;
    digitalWrite(ENABLE, LOW);
    end_f = 1;
  }

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
  
  // ensures function isn't taking too long
  if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
  
  // creates delay of SAMPLE_TIME ms
  while(millis() < (currentTime + SAMPLE_TIME));
  
} // end of loop 
