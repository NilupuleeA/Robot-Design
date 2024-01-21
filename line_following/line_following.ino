#include "line_following_functions.h"

void setup() {
  Serial.begin(9600);

  // Initialize IR sensors
  initializeIRSensors();

  // Initialize Motors
  initializeMotors();
  
  // Give signals after defining pinModes
  shock(); 
}

void loop() {
  delay(500); 
  
  // Move forward while keeping the desired distance from the right side
  pid_line_following();

}
