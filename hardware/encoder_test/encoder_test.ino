// Import motordriver library to make sure there are no pin conflicts
#include "/home/jason/Arduino/libraries/DualVNH5019MotorShield/DualVNH5019MotorShield.h"
#include "/home/jason/Arduino/libraries/DualVNH5019MotorShield/DualVNH5019MotorShield.cpp"
DualVNH5019MotorShield md;

#define ENCODER_OPTIMIZE_INTERRUPTS // used on import
#include "/home/jason/Arduino/libraries/Encoder/Encoder.h" // MODIFY: remove privacy on constructor
#include "/home/jason/Arduino/libraries/Encoder/Encoder.cpp"

Encoder knobLeft(18, 19); // MUST USE AT LEAST ONE EXTERNAL INTERRUPT
Encoder knobRight(20, 21); // MUST USE AT LEAST ONE EXTERNAL INTERRUPT

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(3);
  Serial.println("TwoKnobs Encoder Test:");
  pinMode(13, OUTPUT);
  md.init();
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newRight = knobRight.read();
  newLeft = knobLeft.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}
