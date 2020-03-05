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

char *cmd_list[] = {"mov_f", "mov_b", "turn_l", "turn_r", "spin_l", "spin_r", "base", "shoulder", "elbow", "wrist", "gripper"};
bool flag = false;

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(9600); 
  Serial2.begin(9600); 
  Serial3.begin(9600); 
  Serial.println("Ready to Receive!");
}

void loop()
{
  if (flag == false) {
    flag = true;
    find_matches();
  }
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
      int new_value = Serial2.read();   //after the full name and colon have been received, retrieve the next incoming byte as the command value.
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
  print_command(cmd_index);
}

void print_command(int cmd_count) {
  for (int i = 0; i < cmd_count; i++) {
    Serial.print(cmd_group[i].cmd_name);
    Serial.print(": ");
    Serial.println(cmd_group[i].cmd_value);
  }
}

void parse_command() {
  
}

void find_matches() {
//  char *new_list[sizeof(cmd_list) / sizeof(cmd_list[0])];
  char *new_list[sizeof(cmd_list) / sizeof(cmd_list[0])];
  for (byte i = 0; i < (sizeof(cmd_list) / sizeof(cmd_list[0])); i++) {   //create copy of cmd_list
    new_list[i] = cmd_list[i];
    Serial.println(cmd_list[i]);
  }
  Serial.println("New List!");
  for (byte i = 0; i < (sizeof(new_list) / sizeof(new_list[0])); i++) {   //print the new list
    Serial.println(new_list[i]);
  }
  for (byte i = 0; i < (sizeof(cmd_group) / sizeof(cmd_group[0])); i++) {
    new_list[(sizeof(new_list) / sizeof(new_list[0])) + i] = cmd_group[i].cmd_name;
  }
  Serial.println("New List Plus Commands!");
  for (byte i = 0; i < (sizeof(new_list) / sizeof(new_list[0])); i++) {   //print the new list plus commands from cmd_group
    Serial.println(new_list[i]);
  }
}

void runMotor() {
  motor1.run(125);
  delay(1000);
  motor1.stop();
}
