/*
 * This program demonstrates serial communication between a Raspberry Pi single board computer and a Makeblock MegaPi as a motor controller.
 * The MegaPi is based on an Arduino Mega 2560 board and receives the serial information on Serial2.
 * The MegaPi controls the output of 8 DC motors and the related sensors for each motor (encoder or potentiometer.
 * 
 * Serial information is sent as strings of numerical digits.
 * Each messages consists of a "command code" and a "command value" separated by a colon ":" (e.g. "cmd_code:cmd_value").
 * The command value is sent as a float with one or two trailing zeros (e.g. "100.00").
 * The period character "." acts as a delimiter to seprate individual commands within the message.
 * Any trailing zero(s) from the last command are truncated to one decimal place and appended to the beginning of the next command code as a single zero (e.g. "010")
 * Since the codes are treated as integers, the leading zeros have no effect on recognizing the intended code.
 * The messages are parsed and converted to integers for use in motor control commands.
 */

#include "MeMegaPi.h"

//Define objects for the 8 DC motors
MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);
MeMegaPiDCMotor motor5(PORT3A);
MeMegaPiDCMotor motor6(PORT3B);
MeMegaPiDCMotor motor7(PORT4A);
MeMegaPiDCMotor motor8(PORT4B);

int motorSpeed = 125; /* value: between -255 and 255. */
const byte MOTOR1 = A6;   //assign analog pins to each motor
const byte MOTOR2 = A7;
const byte MOTOR3 = A8;
const byte MOTOR4 = A9;
const byte MOTOR5 = A10;
const byte MOTOR6 = A11;
const byte MOTOR7 = A12;
const byte MOTOR8 = A13;
int encoder1 = 0;         //variables to count encoder disc "ticks" each time the encoder circuitry sends an interrupt pulse. 
int encoder2 = 0;
int encoder3 = 0;
bool lastState1 = false;
bool lastState2 = false;
bool lastState3 = false;
int  clawState  = 0;    //set the initial state of the rear claw. 0=close, 1=open
bool moving = false;
//int encoderTarget = 40;
bool flag = true; //a flag variable for testing purposes

//Define a struct to hold each command:value pair
typedef struct {
  String  cmd_code;
  String  cmd_value;
} CMD;

//Define an array of structs to hold the incoming command group.
//(Array size determines the number of commands that can be sent in one serial message.)
CMD cmd_group[16];

void setup()
{
  Serial.begin(9600);   //start serial port for the Serial Monitor
  Serial2.begin(9600);  //start serial port to receive the incoming data 
  Serial.println("Ready to Receive!");    //signal that the MegaPi is ready to receive data
  pinMode(MOTOR1, INPUT);
  pinMode(MOTOR2, INPUT);
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
  int byte_count = 0;   //track the number of bytes received (also acts as the character index of the received string)
  int cmd_index = 0;    //track the number of commands received in the message
  bool flag2 = false;   //track if storing "code" data (flag=false) or "value" data (flag=true) for the current command
  while (Serial2.available()>0) {     //repeat until the entire message is retrieved
    char new_byte = Serial2.read();   //grab a byte
    Serial.print("new_byte: ");       //display what it is
    Serial.println(new_byte);
    if (new_byte == 58) {   //check for colon character ":" (separator between the command "code" and "value")
      flag2 = true;        //indicate that we have finished retrieving the code. we are now retrieving the value.
      byte_count = 0;     //reset character index for the next incoming string (command value)
    }
    else if (new_byte == 46) {    //check for period character "." (separator between commands)
      cmd_index++;        //update number of commands received   
      flag2 = false; 
      /*NOTE: 
       * Once the period is found, nothing is done to the trailing zero(s) after it.
       * Any trailing zero(s) will be truncated to one decimal place and retrieved as the first character in the code for next command.
       * Because the command codes are treated as integers, a leading zero in the command code doesn't matter.
       * IMPORTANT: Non-zeros after the period "." will cause errors!!
       */
    }
    else {
      if (flag2 == false) {
        cmd_group[cmd_index].cmd_code = cmd_group[cmd_index].cmd_code + new_byte;   //add new digit to command code
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
    Serial.print(cmd_group[i].cmd_code.toInt());    //convert the string for the command code to an integer value
    Serial.print(": ");
    Serial.println(cmd_group[i].cmd_value);
  }
}

void execute_command(int cmd_count) {
  for (int i = 0; i < cmd_count; i++) {     //Loop through each received command code and perform the necessary actions
    switch (cmd_group[i].cmd_code.toInt()){
      case 10:
        Serial.print("Move Forward: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
          move_forward(cmd_group[i].cmd_value.toInt());
        break;
      case 15:
        Serial.print("Move Backward: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
//        motor1.run(-125);
//        motor2.run(-125);
//        delay(cmd_group[i].cmd_value.toInt()*10);
//        motor1.stop();
//        motor2.stop();
        break;
      case 40:
        Serial.print("Rear Claw: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
          move_claw(cmd_group[i].cmd_value.toInt());
        break;
      case 50:
        Serial.print("Shoulder Turning: ");
        Serial.println(cmd_group[i].cmd_value.toInt());
          move_arm(cmd_group[i].cmd_value.toInt());
        break;
      default:
        Serial.println("Command Code Not Found!!");
        break;
    }
  }
  memset(cmd_group, 0, sizeof(cmd_group));    //reset the command group to prepare for the next incoming message
}

void runMotor() {
  motor1.run(125);
  delay(1000);
  motor1.stop();
}

void move_forward(int encoderTarget) {
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);
  Serial.print("Encoder Target: ");
  Serial.println(encoderTarget);
  moving = true;
  motor1.run(motorSpeed);   //LEFT WHEEL
  motor2.run(-motorSpeed);   //RIGHT WHEEL
  while (encoder1 < encoderTarget && encoder2 < encoderTarget) {
    checkEncoders();
  }
  motor1.stop();
  motor2.stop();
  resetEncoders();
  moving = false;
}

void move_claw(int new_state) {   //The "claw" is the REAR grasping mechanism
  Serial.print("Claw Speed: ");
  Serial.println(motorSpeed);
  moving = true;
  if (new_state == 0 && clawState == 1) {
    motor3.run(-motorSpeed);   //CLAW MOTOR: Negative values will CLOSE  the claw.
    clawState = 0;
  }
  else if (new_state == 1 && clawState == 0) {
    motor3.run(motorSpeed);   //CLAW MOTOR: Positive values will OPEN  the claw.
    clawState = 1;
  }
  delay(1500);
  motor3.stop();
  resetEncoders();
  moving = false;
}

void move_arm(int encoderTarget) {
  Serial.print("Arm Speed: ");
  Serial.println(motorSpeed);
  Serial.print("Encoder Target: ");
  Serial.println(encoderTarget);
  moving = true;
  motor4.run(motorSpeed);   //Base Movement. POLARITY SUBJECT TO CHANGE: Negative values rotate base clockwise (when looking top-down at the robot).
  while (encoder3 < encoderTarget && encoder3 < encoderTarget) {
    checkEncoders();
  }
  motor4.stop();
  resetEncoders();
  moving = false;
}

void checkEncoders() {
  /*
   * Increment encoder values one tick per any sensor value change HIGH or LOW.
   * Use "lastState" variable to prevent false retriggers of the same sensor slot.
   * Print the trigger type (HIGH or LOW) and current value.
   */
  //Update Encoder 1: Left Wheel
  if (analogRead(MOTOR1) > 900 && lastState1 == false) {
    lastState1 = HIGH;
    encoder1++;
    Serial.print("WHEEL_L H: ");
    Serial.println(encoder1);
  }
  else if (analogRead(MOTOR1) < 100 && lastState1 == true) {
    lastState1 = LOW;
    encoder1++;
    Serial.print("WHEEL_L L: ");
    Serial.println(encoder1);
  }

  //Update Encoder 2: Right Wheeel
  if (analogRead(MOTOR2) > 900 && lastState2 == false) {
    lastState2 = HIGH;
    encoder2++;
    Serial.print("                  WHEEL_R H: ");
    Serial.println(encoder2);
  }
  else if (analogRead(MOTOR2) < 100 && lastState2 == true) {
    lastState2 = LOW;
    encoder2++;
    Serial.print("                  WHEEL_R L: ");
    Serial.println(encoder2);
  }

    //Update Encoder 3: Base
  if (analogRead(MOTOR3) > 900 && lastState3 == false) {
    lastState3 = HIGH;
    encoder3++;
    Serial.print("                  BASE H: ");
    Serial.println(encoder3);
  }
  else if (analogRead(MOTOR3) < 100 && lastState3 == true) {
    lastState3 = LOW;
    encoder3++;
    Serial.print("                  BASE L: ");
    Serial.println(encoder3);
  }
}

void resetEncoders() {
  encoder1 = 0;
  encoder2 = 0;
}
