/*
 * This program demonstrates basic serial communication between a MegaPi and a Raspberry Pi.
 * The MegaPi receives the serial information on Serial2.
 */

#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor7(PORT4A);
MeMegaPiDCMotor motor8(PORT4B);

//Define a struct to hold each command:value pair
typedef struct {
  String  cmd_name;
  String  cmd_value;
} CMD;

//Define an array of structs to hold the incoming command group
CMD cmd_group[16];

//char *cmd_list[] = {"mov_f", "mov_b", "turn_l", "turn_r", "spin_l", "spin_r", "base", "shoulder", "elbow", "wrist", "gripper"};
bool flag = true;

void setup() 
{
  Serial.begin(9600);   //start serial communication for the Serial Monitor
  Serial2.begin(9600);  //start serial communication to receive the incoming data
  Serial.println("Ready to Receive!");    //signal that the MegaPi is ready to receive data
}

void loop()
{
  if (flag == false) {    //flag gate for testing
    flag = true;
    //do something once
  }
  
  if (Serial2.available()>0) {    //Check for message.
    get_cmd();                    //Process message.
  }   
}


void get_cmd() {
  int byte_count = 0;   //track the index of bytes received (characters received)
  int cmd_index = 0;    //track the number of commands received
  bool flag2 = false;    //track if storing "name=false" or "value=true" data
  while (Serial2.available()>0) {
    char new_byte = Serial2.read();   //grab a byte
    Serial.print("new_byte: ");       //tell us what it is
    Serial.println(new_byte);
    if (new_byte == 58) {   //check for colon character ":" (separator between the command "name" and "value")
      flag2 = true;
      byte_count = 0;     //reset "byte_count" to prepare for the incoming command value
    }
    else if (new_byte == 46) {    //check for period character "." (separator between commands)
      cmd_index++;        //update number of commands received   
      flag2 = false; 
      /*NOTE: 
       * Once the period is found, nothing is done to the zero (or zeroes) after it.
       * Any trailing zero(s) will be retrieved as the first character(s) in the name of next command.
       * Because the command names are treated as integers, the trialing zeros don't matter.
       * Non-zeros after the period "." will cause errors!
       */
    }
    else {
      if (flag2 == false) {
        cmd_group[cmd_index].cmd_name = cmd_group[cmd_index].cmd_name + new_byte;   //add new character to command name
        byte_count++;
      }
      else if (flag2 == true){ 
        cmd_group[cmd_index].cmd_value = cmd_group[cmd_index].cmd_value + new_byte;   //add new digit to command value
        byte_count++;
      }
      delay(10);
    }
  }

//  print_command(cmd_index);   //display the received command group to the Serial Monitor
    execute_command(cmd_index);
}

void print_command(int cmd_count) {
  for (int i = 0; i < cmd_count; i++) {
    Serial.print(cmd_group[i].cmd_name.toInt());    //convert the string for the command code to an integer value
    Serial.print(": ");
    Serial.println(cmd_group[i].cmd_value);
  }
}

void execute_command(int cmd_count) {
  for (int i = 0; i < cmd_count; i++) {
    switch (cmd_group[i].cmd_name.toInt()){
      case 10:
        Serial.print("Move Forward: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
        motor1.run(125);
        motor2.run(125);
        delay(cmd_group[i].cmd_value.toInt()*10);
        motor1.stop();
        motor2.stop();
        break;
      case 15:
        Serial.print("Move Backward: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
        motor1.run(-125);
        motor2.run(-125);
        delay(cmd_group[i].cmd_value.toInt()*10);
        motor1.stop();
        motor2.stop();
        break;
      case 40:
        Serial.print("Base Turning: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
        motor7.run(255);
        delay(cmd_group[i].cmd_value.toInt()*10);
        motor7.stop();
        break;
      case 50:
        Serial.print("Shoulder Turning: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
        motor8.run(-255);
        delay(cmd_group[i].cmd_value.toInt()*10);
        motor8.stop();
        break;
      default:
        Serial.println("Command Code Not Found!!");
        break;
    }
  }
  memset(cmd_group, 0, sizeof(cmd_group));
}

void runMotor() {
  motor1.run(125);
  delay(1000);
  motor1.stop();
}
