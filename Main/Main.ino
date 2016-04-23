#include "Config.h"

RH_RF69 rf69 (4);

ServoTimer2 motors[4];

long ledTimeStamp = 0;
long spinTimeStamp = 0; 

const int maxThrottle = 30;
const double spinTime = 15000;
const bool sweep = true;

enum MotorState {
  startSpinning,
  spinning,
  idle
};

enum ArmLEDState {
  on,
  off
};

ArmLEDState armLEDState;
MotorState motorState;

void setup() {
  analogReference(EXTERNAL);
  
  pinMode(BUTTON, INPUT);
  
  initComm();
  initIMU();
  initLED();
  initPID();
  initMotors();

  motorState = idle;
  armLEDState = off;
  
  setThrottleAll(100);
  delay(3000);
  setThrottleAll(-1);

}

void loop() {

  updateLED();

  updateOrientation();
  
  updateComm();
  
  updatePID();

  updateMotors();
  
}
