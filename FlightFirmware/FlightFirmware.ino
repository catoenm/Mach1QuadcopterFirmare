#include "Config.h"
#include <Servo.h>
#include <PID_v1.h>

int setPoint [3];
int throttle;
Servo PROP_1;
Servo PROP_2;
Servo PROP_3;
Servo PROP_4;


void setup() {
  // put your setup code here, to run once:
  setPoint [0] = 0; //X
  setPoint [1] = 0; //Y
  setPoint [2] = 0; //Z
  
  throttle = 0;
  
  PROP_1.attach(ESC1);
  PROP_2.attach(ESC2);
  PROP_3.attach(ESC3);
  PROP_4.attach(ESC4);
  
  initControllerInput();
  initIMU();
  initLED();
  initPID();
  
}

void loop() {
  
  updateOrientation();
  
  updateControllerInput();
  
  updatePID();
  
  updateLED();
  
}
