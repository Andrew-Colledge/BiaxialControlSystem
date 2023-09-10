// Toronot Metroploital Univeristy - Aerospace Engineering Department
// Project: Biaxial Testing Apparatus Control System
// Created by: Andrew Colledge

// The following code is used to control the stepper motors and load cell of the biaxial testing apparatus.
// This code works in conjunction with a required MATLAB program.
// Uploaded this code to the Arduino Uno, Then run the MATLAB UI program

// August 2023

#include <AccelStepper.h>
#include "HX711.h"                  
#include <math.h>
#define calibration_factor -7050.0  //This value is obtained using the SparkFun_HX711_Calibration sketch
HX711 scaleX;
HX711 scaleY;
AccelStepper myStepperX(AccelStepper::DRIVER,3,4);
AccelStepper myStepperY(AccelStepper::DRIVER,8,9);
int x = 0;
int xDirection = -1;  //0 = Compression, 1 = Tension
int yDirection = -1;  //0 = Compression, 1 = Tension
float xStrainRate = 0.0;
float yStrainRate = 0.0;
float dataPointNum = 0;
String Message = "";
float dispX = 0;
float dispY = 0;
String adjustDir = "";
String XP = "";
String XN = "";
String YP = "";
String YN = "";
String adjustSpeed = "";
float Speed = 0.0;
bool shouldRun = false;
bool shouldParse = false;
bool shouldAdjust = false;

//============
//Pin set up
//stepper X Pul:3 Dir:4 OPTO:5
//stepper Y Pul:8 Dir:9 OPTO:10
//Load Cell X DOUT/DAT:0 SCK/CLK:1 VDD AND VCC:2
//Load Cell Y DOUT/DAT:11 SCK/CLK:12 VDD AND VCC:13
//============

void setup() 
{
  Serial.begin(9600);
  //Pin setup for 5 V output
  pinMode (2, OUTPUT);
  pinMode (5, OUTPUT);
  pinMode (10, OUTPUT);
  pinMode (13, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(13, HIGH);
  //Setup loadcell
  scaleX.begin(1,0); // DOUT = 13, SCK = 12
  scaleY.begin(12,11);
  scaleX.set_scale(-7050.0);  //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scaleY.set_scale(-7050.0);
  scaleX.tare();  //reset the scale to 0
  scaleY.tare();
  //Setup stepper motor
  myStepperX.setMaxSpeed(100000);
  myStepperY.setMaxSpeed(100000);
  myStepperX.setSpeed(0);//initialize X speed to zero
  myStepperY.setSpeed(0);//initialize Y speed to zero
  shouldParse = true;
  shouldRun = false;
  shouldAdjust = false;
}
//============
void loop() 
{
  while (Serial.available() >= 2) 
  {
    Message = Serial.readStringUntil('\n');
    delay(2);
    Message.trim();
  }
  if (Message.length() == 2 && !shouldAdjust)//adjust Stepper position before test starts
  {
    adjustStepper();
  }
  if (Message.length() > 10 && shouldParse)//parses incoming Matlab message and starts test
  {
    parseData();
  }
  else if (Message.length() == 5)//stops motors
  {
    stop();
  }
  else if (Message.length() == 6) //return to home
  {
    returnToHome();
  }
  else if (Message.length() == 7) //sets home position
  {
    myStepperX.setCurrentPosition(0);
    myStepperY.setCurrentPosition(0);
  }
  if (shouldRun)
  {
    step();
  }
  if (shouldAdjust)
  {
    adjustStepperRun();
  }
}
//============
void adjustStepper()
{
  //parse stepper adjustment message
  adjustDir = Message;
  adjustSpeed = Serial.readStringUntil('\n');
  String adjustSpeedCopy = adjustSpeed;
  const char* MC = adjustSpeedCopy.c_str();
  char* Segment;              // this is used by strtok() as an index
  Segment = strtok(MC, "");  
  Speed = atof(Segment); 
  if (adjustDir == "XP") {myStepperX.setSpeed(Speed);}
  if (adjustDir == "XN") {myStepperX.setSpeed(-1*Speed);}
  if (adjustDir == "YP") {myStepperY.setSpeed(Speed);}
  if (adjustDir == "YN") {myStepperY.setSpeed(-1*Speed);}
  shouldAdjust = true;
}
//============
void adjustStepperRun()
{
  myStepperX.runSpeed();
  myStepperY.runSpeed();
}
//============
void parseData() 
{
  //Parse MATLAB Message
  String messageCopy = "";
  messageCopy = Message;
  const char* MC = messageCopy.c_str();
  char* Segment;              // this is used by strtok() as an index
  Segment = strtok(MC, ",");  // get the first part - the string
  xDirection = atoi(Segment); //X tension or compression

  Segment = strtok(NULL, ",");  // this continues where the previous call left off
  xStrainRate = atof(Segment);  //X strainrate

  Segment = strtok(NULL, ",");
  yDirection = atoi(Segment);  //Y tension or compression

  Segment = strtok(NULL, ",");
  yStrainRate = atof(Segment);  //Y strainrate

  Segment = strtok(NULL, ","); //data point per mm
  dataPointNum = atoi(Segment);

  stepperSetup(); //sets up the two stepper motors

  shouldRun = true;
  shouldParse = false;
}
//============
void stop()
{
  myStepperX.stop();
  myStepperY.stop();
  shouldRun = false;
  shouldAdjust = false;
  shouldParse = true;
}
//============
void returnToHome()
{
  myStepperX.setAcceleration(4000);
  myStepperY.setAcceleration(4000);
  myStepperX.runToNewPosition(0); //runs X stepper to home position(block code)
  myStepperY.runToNewPosition(0); //runs Y stepper to home position(block code)
}
//============
void step()
{
  myStepperX.runSpeed();
  myStepperY.runSpeed();
  if (myStepperX.speed() != 0)
  {
    x = abs(myStepperX.currentPosition());
  }
  if (myStepperY.speed() != 0)
  {
    x = abs(myStepperY.currentPosition());
  }
  if (myStepperX.speed() != 0 && myStepperY.speed() != 0)
  {
    x = (abs(myStepperX.currentPosition())+abs(myStepperY.currentPosition()))/2;
  }
  if (fmod(x,dataPointNum) == 0)
  {
    Serial.println(scaleX.get_units(), 1); //send X load to Matlab
    Serial.println(scaleY.get_units(), 1); //send Y load to Matlab
    // Serial.println(0); //stand in data point for X load
    // Serial.println(0); //stand in data point for Y load
    dispX = myStepperX.currentPosition();
    Serial.println(dispX); //send X displacment to Matlab
    dispY = myStepperY.currentPosition();
    Serial.println(dispY); //send Y displacment to Matlab
  }
}
//============
void stepperSetup()
{
  //X stepper
  if (xDirection == 1) { myStepperX.setSpeed(xStrainRate); }
  if (xDirection == 0) { myStepperX.setSpeed(xStrainRate*-1); }
  //Y stepper
  if (yDirection == 1) { myStepperY.setSpeed(yStrainRate); }
  if (yDirection == 0) { myStepperY.setSpeed(yStrainRate*-1); }

}