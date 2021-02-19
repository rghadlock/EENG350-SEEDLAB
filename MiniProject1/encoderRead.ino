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
int resetButton = 5;

//int leftA = 3;
//int leftB = 5;
int counterRight = 0;
//int counterLeft = 0;
double radRight = 0;


void encoderISR(void){//ISR thats triggered when Pin A changes
  
  if(digitalRead(rightA) != digitalRead(rightB)){//if A and B are different after A changes its moving clockwise
    counterRight++;//cw
  }
  else{
    counterRight--;//ccw
  }
  radRight = ((double)counterRight * 6.283)/30;
  Serial.println(radRight);
}

//void leftISR(void){//ISR thats triggered when Pin A changes
//
//  if(digitalRead(leftA) != digitalRead(leftB)){//if A and B are different after A changes its moving clockwise
//    counterLeft++;//cw
//  }
//  else{
//    counterLeft--;//ccw
//  }
//}
void setup() {
  //begin serial communication
  Serial.begin(9600);
  //Assign pins as Inputs
  pinMode(rightA, INPUT);
  pinMode(rightB, INPUT);
  pinMode(resetButton, INPUT);
//  pinMode(leftA, INPUT);
//  pinMode(leftB, INPUT);
  //Set Up ISR
  attachInterrupt(digitalPinToInterrupt(rightA), encoderISR, RISING);
 // attachInterrupt(digitalPinToInterrupt(leftA), leftISR, RISING);

}

void loop() {
  //ISR will run automatically
  if(digitalRead(resetButton) == HIGH){
      counterRight = 0;
      radRight = 0;
  }
  
 
  

}
