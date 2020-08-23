//Copyright 2020 James Beadle
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.

// Stepper Motor Control Code with Serial Communication for Arduino
// James Beadle, May 2020
// This code listens for serial commands, and safely controls 3 servo motors to move to specified positions

//what features should be activated
#define MOTORSUPPORT
#define RXSERIALSUPPORT
#define TXSERIALSUPPORT
//#define DEBUG
//#define SUPPORTDISPLAY


//add OLED display support for debug
#ifdef SUPPORTDISPLAY
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
int textoffset = 0;
#endif

//pin allocations
#define steppinx 3
#define dirpinx 2
#define steppiny 5
#define dirpiny 4
#define steppinz 7
#define dirpinz 6
#define limitx 9
#define limity 8
#define limitz 10

//serial variables
int rxpacketsize = 9;
unsigned char rxpacket[9] = {'b',0,0,0,0,0,0,0,'e'};
int rxcount = 0;
unsigned char rxbegin = 'b';
unsigned char rxend = 'e';
unsigned char commandid = 'd';
bool fullcommand = false;
unsigned char nextchar;

//control variables
int discretetime = 5;
int motorsteptime = 1;
float accelerationsteps = 10;
int dirpin[3] = {dirpinx, dirpiny, dirpinz};
int steppin[3] = {steppinx, steppiny, steppinz};
int targetpositions[3];
int currentpositions[3] = {0, 0, 0};
int oldpositions[3] = {0, 0, 0};
int stepperiod[3] = {1, 1, 1};
int movedelay[3] = {1, 1, 1};
bool movemotor[3] = {false, false, false};
int delayadjust = 0;
bool commandinterupted = false;
bool commandcomplete = true;

//calibrate variables
bool calibrateFlag = false;
bool calibrate = false;
int limitPins[3] = {limitx, limity, limitz};
bool failureRecovery = false;
bool motorCalibrated[3] = {false, false, false};


void setup() 
{
  //start serial communication
  Serial.begin(57600);

#ifdef MOTORSUPPORT
  //set pin functions
  pinMode(steppinx, OUTPUT);
  pinMode(dirpinx, OUTPUT);
  pinMode(steppiny, OUTPUT);
  pinMode(dirpiny, OUTPUT);
  pinMode(steppinz, OUTPUT);
  pinMode(dirpinz, OUTPUT);
  pinMode(limitx, INPUT_PULLUP);
  pinMode(limity, INPUT_PULLUP);
  pinMode(limitz, INPUT_PULLUP);
  pinMode(13, OUTPUT);
#endif

#ifdef SUPPORTDISPLAY
  //set up OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(100);
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(5,1);
  display.print("Start: ");
#endif
}

//delete all the current recorded data from serial
void clearrxpacket()
{
  for (int i = 0; i < (rxpacketsize+1); i++)
  {
    rxpacket[i] = 0;
  }
  return;  
}

//report an error to the OLED display (debug)
void error(char error)
{
#ifdef SUPPORTDISPLAY
  display.print(error);
  display.print(' ');
  display.display();
#endif 
}

//read a single character from the serial buffer
void serialread()
{
  //taking data in 
  //recieve protocol: [b, id, xpos, xpos2, ypos, ypos2, zpos, zpos2, e]
  //send protocol: [b, id, read/complete/fail, e]
  if (Serial.available() > 0)
  {
    nextchar = Serial.read();
    //if its the start of a new command
    if (nextchar == rxbegin)
    {
      error('b');
      clearrxpacket();
      rxpacket[0] = rxbegin;
      rxcount++;
    }
    //if its a request for calibration
    else if ((nextchar == rxend) && (rxcount == 3))
    {
      if (rxpacket[2] == 'c')
      {
        serialwrite('r');
        commandid = rxpacket[1];
        rxpacket[3] = rxend;
        calibrateFlag = true;
        rxcount = 0;
      }
      else
      {
        clearrxpacket();
        rxcount = 0;        
      }
    }
    //if its the end of a new command
    else if ((nextchar == rxend) && (rxcount == (rxpacketsize-1)))
    {
      error('e');
      serialwrite('r');
      rxpacket[rxpacketsize-1] = rxend;
      fullcommand = true;
      rxcount = 0;
    }
    //if the command ends early
    else if ((nextchar == rxend) && (rxcount != (rxpacketsize-1)))
    {
      error('s');
      clearrxpacket();
      rxcount = 0;
    }
    //if the command ends late
    else if (rxcount == (rxpacketsize-1))
    {
      serialwrite('a');
      error('l');
      clearrxpacket();
      rxcount = 0;     
    }
    //if new data is sent
    else
    {
      error(nextchar);
      rxpacket[rxcount] = nextchar;
      rxcount++;
    }
  }  
}

//write a command to serial
void serialwrite(unsigned char message)
{
#ifdef TXSERIALSUPPORT
  byte buf[4] = {'b',commandid, message, 'e'};
  Serial.write(buf, 4);
#endif
}

//use the serial data to write new target positions
void resettargetpositions()
{
  commandcomplete = false;
  //scrape data   
  for (int i = 0; i<3; i++)
  {
    oldpositions[i] = currentpositions[i];
    targetpositions[i] = rxpacket[2*i+2]*64 + rxpacket[2*i+3];
  }
  commandid = rxpacket[1];  
}

//work out of any of the motors should move this step
void configureSpeed()
{
  for (int i = 0; i<3; i++)
  {
    if (currentpositions[i] != targetpositions[i])
    {
      stepperiod[i] = 1;
      //if the motors position is close to its start position: travel slower
      if (accelerationsteps > abs(oldpositions[i] - currentpositions[i]))
      {
        stepperiod[i] += accelerationsteps - abs(currentpositions[i] - oldpositions[i]);
      }
      //if the motors position is close to its end position: travel slower
      if (accelerationsteps > abs(targetpositions[i] - currentpositions[i]))
      {
        stepperiod[i] += accelerationsteps - abs(currentpositions[i] - targetpositions[i]);
      }
      //if the motors step period has passed
      if (movedelay[i] >= abs(stepperiod[i]))
      {
        movemotor[i] = true;
        movedelay[i] = 1;
      }
      else
      {
        movedelay[i] += 1;
      }      
    }
  } 
}

//move any motor which needs moving in the required direction
void stepmotors()
{
  //work out which direction each motor should move in
  int motorDirection[3] = {0,0,0};
  for (int i = 0; i<3; i++)
  {
    if (targetpositions[i] < currentpositions[i])
    {
      motorDirection[i] = -1;
    }
    else if (targetpositions[i] >= currentpositions[i])
    {
      motorDirection[i] = 1;
    }    
  }
#ifdef MOTORSUPPORT
  //set the direction of each motor
  for(int i = 0; i < 3; i++)
  {
    if (motorDirection[i] == 1)
    {
      digitalWrite(dirpin[i], LOW);
    }
    else if (motorDirection[i] == -1)
    {
      digitalWrite(dirpin[i], HIGH);
    }    
  }
  //move motors 1 step
  for(int i = 0; i < 3; i++)
  {
    if (movemotor[i] == true)
    {
      digitalWrite(steppin[i], HIGH);
      currentpositions[i] += motorDirection[i];    
    }      
  }
  delay(motorsteptime);
  for(int i = 0; i < 3; i++)
  {
    if (movemotor[i] == true)
    {
      digitalWrite(steppin[i], LOW);
      movemotor[i] = false;   
    }      
  }  
  delay(motorsteptime);
  
#else
  //if motors are not set to move, update their position
  for(int i = 0; i < 3; i++)
  {  
    if (movemotor[i] == true)
    {
      currentpositions[i] += motorDirection[i]; 
      movemotor[i] = false;    
    }
  }   
  delay(2*motorsteptime);
#endif
}


//main loop
void loop() 
{
  //read one byte from serial
#ifdef RXSERIALSUPPORT
  serialread();
#endif

  //see if the motors need calibrating
  if (calibrateFlag == true)
  {
    calibrateFlag = false;
    calibrate = true;
    commandcomplete = false;
    for (int i = 0; i<3; i++)
    {
      //move the motors until they hit a limit switch
      targetpositions[i] = -10000;
      oldpositions[i] = currentpositions[i];
    }
  }
  //see if the motor is being calibrated
  if (calibrate == true)
  {
    for(int i = 0; i<3; i++)
    {
      //test if a limit switch has been reached
      if ((digitalRead(limitPins[i]) == LOW) and (motorCalibrated[i] == false))
      {
        //move the motor back 100 steps
        motorCalibrated[i] = true;
        currentpositions[i] = -100;
        targetpositions[i] = 0;        
      }
    }   
  }
  
  //if a new command is recieved
  if (fullcommand == true)
  {
    fullcommand = false;
    //if the current command isn't complete
    if (commandcomplete == false)
    { 
      commandinterupted = true;
      //set a new target position 'accelerationsteps' away from the current position
      for (int i = 0; i<3; i++)
      {
        if (accelerationsteps < abs(targetpositions[i] - currentpositions[i]))
        {
          int dir = ((targetpositions[i]-currentpositions[i])/abs(targetpositions[i]-currentpositions[i]));
          targetpositions[i] = currentpositions[i] + (accelerationsteps)*dir;
        }   
      }
    }
    else
    {
      //set up the new target positions
      resettargetpositions();
    }
  }
  if (commandinterupted == true)
  {
    if (commandcomplete == true)
    { 
      //run only if the system has stopped safely
      commandinterupted = false;
      //set up the new target positions
      resettargetpositions();
    }
  }

  //set which motors should move this cycle
  configureSpeed();
  //adjust the delay of the main loop to be constant
  if ((movemotor[0] == true) || (movemotor[1] == true) || (movemotor[2] == true))
  {
    delayadjust = 2*motorsteptime;
  }
  else 
  {
    delayadjust = 0;
  }

  //move the required motors this cycle
  if (commandcomplete == false)
  {
    stepmotors();
#ifdef SUPPORTDISPLAY
    //display each motors current position on the OLED display (debug)
    display.clearDisplay();
    display.setCursor(5,1);
    display.print(currentpositions[0]);
    display.setCursor(5,10);
    display.print(currentpositions[1]);
    display.setCursor(5,20);
    display.print(currentpositions[2]);
    display.setCursor(30,1);
    display.print(targetpositions[0]);
    display.setCursor(30,10);
    display.print(targetpositions[1]);
    display.setCursor(30,20);
    display.print(targetpositions[2]);    
    display.display();  
#endif
    //test if the current command has been complete
    commandcomplete = true; 
    for (int i = 0; i < 3; i++) 
    {
      if (targetpositions[i] != currentpositions[i])
      {
        commandcomplete = false;
      }
    }
    //if the current command is complete
    if (commandcomplete == true)
    {
      serialwrite('c');

      //for when the motors are calibrated
      for(int i=0; i<3;i++)
      {
        motorCalibrated[i] = false;
      }
      failureRecovery = false;
      calibrate = false;
    }
  }


  //test if a limit has been reached
  for (int i=0; i<3; i++)
  {
    if ((digitalRead(limitPins[i]) == LOW) && (calibrate == false) && (failureRecovery == false))
    {
      //'bounce' each motor back 100 steps
      if (targetpositions[i] > currentpositions[i])
      {
        oldpositions[i] = currentpositions[i] + 100;
        targetpositions[i] = currentpositions[i] - 100;
      }
      else
      {
        oldpositions[i] = currentpositions[i] - 100;
        targetpositions[i] = currentpositions[i] + 100;
      }
      //stop moving every other motor
      for (int j = 0; j < 3; j++)
      {
        if (i != j)
        {
          oldpositions[j] = currentpositions[j];
          targetpositions[j] = currentpositions[j];
        }
      }
      //send over serial there has been an error
      serialwrite('f');
      failureRecovery = true;
      break;
    }
  } 

  //debug every position over serial
#ifdef DEBUG
  Serial.print(currentpositions[0]);
  Serial.print(",");
  Serial.print(currentpositions[1]);
  Serial.print(",");
  Serial.print(currentpositions[2]);
  Serial.print(",");
  Serial.print(targetpositions[0]);
  Serial.print(",");
  Serial.print(targetpositions[1]);
  Serial.print(",");
  Serial.print(targetpositions[2]);
  Serial.println(" ");
#endif
  //cycle delay
  delay(discretetime - delayadjust);
}
