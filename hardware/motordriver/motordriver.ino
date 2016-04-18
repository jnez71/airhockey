/*
Program to be run on an arduino with a Dual-VNH5019 Motor Shield.
Listens for serial packet to set motor efforts.
Sends out serial packet with angular odometry from encoders.
*/

///////////////////////////////////////////////// DEPENDENCIES

// Motor driver library
// <https://github.com/pololu/dual-vnh5019-motor-shield> & <https://www.pololu.com/product/2507>
#include "/home/jason/Arduino/libraries/DualVNH5019MotorShield/DualVNH5019MotorShield.h"
#include "/home/jason/Arduino/libraries/DualVNH5019MotorShield/DualVNH5019MotorShield.cpp"

// Encoder library
// <https://www.pjrc.com/teensy/td_libs_Encoder.html> & <http://www.cui.com/product/resource/amt10.pdf>
#define ENCODER_OPTIMIZE_INTERRUPTS // used on import
#include "/home/jason/Arduino/libraries/Encoder/Encoder.h" // MODIFY: remove privacy on constructor
#include "/home/jason/Arduino/libraries/Encoder/Encoder.cpp"

///////////////////////////////////////////////// DEFINITIONS

// Motor driver
DualVNH5019MotorShield md;

// NOT NEEDED: do conversion outside of arduino instead
// Encoder tics to degrees:
// (revs/tic) * (degs/rev) = 1/2048 * 360
// #define ENC_SCALE 0.17578125

// Scale factor from effort% to cmd points
#define CMD_SCALE -4

// Communication management
#define ENC_REQUEST '1'
#define CMD_REQUEST '2'

///////////////////////////////////////////////// VARIABLES

// Motor commands
long cmd_x = 0;
long cmd_y = 0;

// Angular odometry
long odom_x = 0;
long odom_y = 0;

// Encoder objects
Encoder enc_x(18, 19); // MUST USE AT LEAST ONE EXTERNAL INTERRUPT
Encoder enc_y(20, 21); // MUST USE AT LEAST ONE EXTERNAL INTERRUPT

// Communication management
char request = '0';

///////////////////////////////////////////////// FUNCTIONS

void send_long(long arg)
{
  // Get access to the float as a byte-array
  byte* data = (byte*) &arg;
  // Write the data to serial
  Serial.write(data, sizeof(arg));
}

void stopIfFault()
{
  // Stop and alert if first motor faulted
  if (md.getM1Fault()){
    Serial.println("M1 fault");
    while(true);
  }
  // Stop and alert if second motor faulted
  if (md.getM2Fault()){
    Serial.println("M2 fault");
    while(true);
  }
}

///////////////////////////////////////////////// SET-UP ARDUINO

void setup()
{
  // Open serial port and override timeout duration
  Serial.begin(9600);
  Serial.setTimeout(3);
  // Set LED indicator pin to output
  pinMode(13, OUTPUT);
  // Initialize md library
  md.init();
  // Make sure md output is zero
  md.setM1Speed(0);
  md.setM2Speed(0);
}

///////////////////////////////////////////////// MAIN

void loop()
{
  // Get latest encoder value
  odom_x = enc_x.read();// * ENC_SCALE;
  odom_y = enc_y.read();// * ENC_SCALE;

  // Read request from computer
  while(request == '0'){
    if(Serial.available() > 0){
      request = Serial.read();
    }
  }

  // Send angular odometry
  if(request == ENC_REQUEST){
    send_long(odom_x);
    send_long(odom_y);
  }

  // Receive motor commands
  else if(request == CMD_REQUEST){
    // First int has x command
    cmd_x = Serial.parseInt() * CMD_SCALE;
    md.setM1Speed(cmd_x);
    stopIfFault();
    // Second int has y command
    cmd_y = Serial.parseInt() * CMD_SCALE;
    md.setM2Speed(cmd_y);
    stopIfFault();
    // Acknowledge that we received
    Serial.write('0');
  }

  // Reset request manager
  request = '0';
}
