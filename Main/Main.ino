#include "Config.h"

//Cyclone Robotics Main Flight File

void setup() {
  Serial.begin(9600);
  
  analogReference(EXTERNAL);
  
  pinMode(BUTTON, INPUT);
  
  initComm();
  initIMU();
  initLED();
  initPID();
  initMotors();
  
  armLEDState = off;
  
  setThrottleAll(100);
  delay(3000);
  setThrottleAll(-1);

}

void loop() {

  //Process User Commands
  updateComm();

  //Process R/P/Y
  updateOrientation();

  //Compute PID Outputs
  updatePID();

  //Set Motor Throttles
  updateMotors();

  //Run LED S/M
  updateLED();

  motor0Out;
  motor1Out;
  motor2Out;
  motor3Out;
  
}
