/*
 * This program receives serial communication and displays which serial port is being used to receive the data.
 * Also displays the data received.
 */

#include "MeMegaPi.h"


int data = 0;
const int COMMAND_LENGTH = 2; //The number of elements in a command. RPi and MegaPi MUST use the same value.
byte commandBuffer[COMMAND_LENGTH]; //create storage buffer for the command message.

void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  //Start desired serial channel (MegaPi has more than 1) at the desired baud rate.
  Serial.begin(9600);
}

void loop()
{
  //Receive from Raspberry Pi
  if (Serial.available()>0){
    Serial.readBytes(commandBuffer, COMMAND_LENGTH);  //Store command elements into the commandBuffer
    //Identify and execute command
    switch(commandBuffer[0]){
      case 217:
        blinkLED(3);
      break;
      case 114:
        blinkLED(2);
      break;
      default:
        blinkLED(1);
      break;
    }

    //Send reply back to Raspberry Pi
    for (int i = 0; i < COMMAND_LENGTH; i++) {    //For each command element...
      Serial.write(commandBuffer[i]);   //Repeat the command element back to the RPi 
    }
  }
}

void blinkLED(int blinkCount)
{
  int duration = 400;
  for(int i=0; i<blinkCount; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(duration);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(duration);
  }
}
