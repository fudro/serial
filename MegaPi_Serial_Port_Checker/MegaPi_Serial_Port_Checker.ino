/*
 * This program receives serial communication and displays which serial port is being used to receive the data.
 * Also displays the data received.
 */

#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1A);

int message = 0;
int dataport0 = 0;
int dataport1 = 0;
int dataport2 = 0;
int dataport3 = 0;

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(9600); 
  Serial2.begin(9600); 
  Serial3.begin(9600); 
  Serial.println("Starting Serial Port Checker!");
}

void loop()
{
  if (Serial.available()>0){
    // read serial port "0"
    dataport0 = Serial.read();
    Serial.print("Serial Port 0 (default USB): ");
    Serial.println(dataport0);
  }
  
  if (Serial1.available()>0){
    // read serial port "1"
    dataport1 = Serial1.read();
    Serial.print("Serial Port 1: ");
    Serial.println(dataport1);
  }
  
  if (Serial2.available()>0){
    // read serial port "2"
    dataport2 = Serial2.read();
    Serial.print("Serial Port 2: ");
    Serial.println(dataport2);
  }
  
  if (Serial3.available()>0){
    // read serial port "3"
    dataport3 = Serial3.read();
    Serial3.print("Serial Port 3: ");
    Serial3.println(dataport3);
  }

  if (dataport0 != 0) {
    runMotor();
    delay (5000);
    dataport0 = 0;
  }
  else if (dataport1 != 0) {
    runMotor();
    delay (5000);
    dataport1 = 0;
  }
  else if (dataport2 != 0) {
    runMotor();
    delay (5000);
    dataport2 = 0;
  }
  else if (dataport3 != 0) {
    runMotor();
    delay (5000);
    dataport3 = 0;
  }
}

void runMotor() {
  motor1.run(125);
  delay(1000);
  motor1.stop();
}
