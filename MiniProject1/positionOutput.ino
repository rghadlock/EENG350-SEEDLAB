/*Jacob Bommersbach 
 * EENG 350
 * Assignment 2 Localization
 * Robot Position Locator
 * This Program will use 2 digital encoders 
 * This program will read 2 digital encoders and display the calculated
 * X position Y position and the angle for a simulated robot
 * with the 2 encoders acting as the wheels of the robot
 * Connect the ground and Vcc of the Encoders to the Arduino, and connect the A output to Pin 2 
 * and the B output to Pin 4 for the right encoder and A to 3, B to 5 for the Left encoder
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

double angVelRight = 0;
double angVelLeft = 0;
double r = .05;
double b = .1;
double xPos = 0;
double yPos = 0;
double xPosOld = 0;
double yPosOld = 0;
double phi = 0;
double phiOld = 0;
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
  //Set the Old values to the run before the time delay
  xPosOld = xPos;
  yPosOld = yPos;
  phiOld = phi;
  radLeftOld = radLeftNew;
  radRightOld = radRightNew;
  //Delay to allow reading encoders and changes
  delay(100);
  //covert encoder count values to radians and change in radians to angular velocity
  radRightNew = ((double)counterRight * 6.283)/30;
  angVelRight = (radRightNew - radRightOld)*10;
  radLeftNew = ((double)counterLeft * 6.283)/30;
  angVelLeft = (radLeftNew - radLeftOld)*10;

  //Use Euler approximation for finding x and y posistion and the angle
  xPos = xPosOld +(0.05)*cos(phiOld)*( angVelRight*r + angVelLeft*r);
  yPos = yPosOld +(0.05)*sin(phiOld)*( angVelRight*r + angVelLeft*r);
  phi = phiOld +(.1)*(r/b)*(angVelLeft*r - angVelRight*r);
  //Print the value seperated by tabs 
  Serial.print(xPos); 
  Serial.print("\t");  
  Serial.print(yPos);
  Serial.print("\t");
  Serial.print(phi);
  Serial.print("\t");
  Serial.println(); 

}
