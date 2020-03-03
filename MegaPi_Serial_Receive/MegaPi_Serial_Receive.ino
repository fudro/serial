/*
 * This program demonstrates basic serial communication between a MegaPi and a Raspberry Pi.
 */

#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1A);

//Define a struct to hold each command:value pair
typedef struct {
  char cmd_name[16];
  int  cmd_value;
} CMD;

//Define an array of structs to hold the incoming command group
CMD cmd_group[16];

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(9600); 
  Serial2.begin(9600); 
  Serial3.begin(9600); 
  Serial.println("Starting Serial Test!");
}

void loop()
{
  if (Serial2.available()>0) {
    get_cmd();
  }   
}

void get_cmd() {
  int byte_count = 0;   //track each received byte to rebuild the full command name
  int cmd_index = 0;    //track the number of commands received
  while (Serial2.available()>0) {
    char new_byte = Serial2.read();
    Serial.print("new_byte: ");
    Serial.println(new_byte);
    if (new_byte == 58) {              //the ":" character (ASCII 58) denotes the end of the command name (based on the Python dictionary format of "command:value")
      int new_value = Serial2.read();   //after the full name has been received, retrieve the command value from the next incoming byte.
      Serial.print("cmd_value: ");
      Serial.println(new_value);
      cmd_group[cmd_index].cmd_value = new_value;
      cmd_index++;        //update number of commands received
      byte_count = 0;     //reset "byte_count" to prepare for next incoming command name
    }
    else {
      cmd_group[cmd_index].cmd_name[byte_count] = new_byte;   //update received command name
      byte_count++;
    }
    delay(10);
  }
  printCommands(cmd_index);
}

void printCommands(int cmd_count) {
  for (int i = 0; i < cmd_count; i++) {
    Serial.print(cmd_group[i].cmd_name);
    Serial.print(": ");
    Serial.println(cmd_group[i].cmd_value);
  }
}

void runMotor() {
  motor1.run(125);
  delay(1000);
  motor1.stop();
}
