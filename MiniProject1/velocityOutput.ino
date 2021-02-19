/*Jacob Bommersbach 
 * EENG 350
 * Assignment 2 Localization
 * Robot Position Locator
 * This Program will use 2 digital encoders 
 * This program will read 2 digital encoders and display the calculated
 * Velocity of either wheels for a simulated robot
 * with the 2 encoders acting as the wheels of the robot
 * Connect the ground and Vcc of the Encoders to the Arduino, and connect the A output to Pin 2 
 * and the B output to Pin 4 for the right encoder and A to 3, B to 5 for the Left encoder
 * The serial monitor will output in the columns time, velocity Left, and velocity right respectively
 */
//initalize pins and variables
int rightA = 2;
int rightB = 4;
int leftA = 3;
int leftB = 5;
int counterRight = 0;
int counterLeft = 0;
double radRightNew = 0;
double radLeftNew = 0;
double radRightOld = 0;
double radLeftOld = 0;
double velRight = 0;
double velLeft = 0;
double angVelRight = 0;
double angVelLeft = 0;
double r = .05;
double b = .1;
double t = 0;


void rightISR(void){//ISR thats triggered when Pin A changes
  
  if(digitalRead(rightA) != digitalRead(rightB)){//if A and B are different after A changes its moving clockwise
    counterRight++;//cw
  }
  else{
    counterRight--;//ccw
  }
}

void leftISR(void){//ISR thats triggered when Pin A changes

  if(digitalRead(leftA) != digitalRead(leftB)){//if A and B are different after A changes its moving clockwise
    counterLeft++;//cw
  }
  else{
    counterLeft--;//ccw
  }
}
void setup() {
  //begin serial communication
  Serial.begin(9600);
  //Assign pins as Inputs
  pinMode(rightA, INPUT);
  pinMode(rightB, INPUT);
  pinMode(leftA, INPUT);
  pinMode(leftB, INPUT);
  //Set Up ISR
  attachInterrupt(digitalPinToInterrupt(rightA), rightISR, RISING);
  attachInterrupt(digitalPinToInterrupt(leftA), leftISR, RISING);

}

void loop() {
  //ISR will run automatically
  
  //Switch Old and New values
  radLeftOld = radLeftNew;
  radRightOld = radRightNew;

  delay(100);
  //calculate The angular velocity  
  radRightNew = ((double)counterRight * 6.283)/30;
  angVelRight = (radRightNew - radRightOld)*10;
  radLeftNew = ((double)counterLeft * 6.283)/30;
  angVelLeft = (radLeftNew - radLeftOld)*10;
  //Convert Angular and Tangential Velocity
  velRight = angVelRight*r;
  velLeft = angVelLeft*r;
  //Print The outputs seperated by tabs
  Serial.print(t);  
  Serial.print("\t");        
  Serial.print(velLeft);
  Serial.print("\t");
  Serial.print(velRight);
  Serial.print("\t");
  Serial.println();
  t += 0.1; 

}
