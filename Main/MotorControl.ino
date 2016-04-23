
//Cyclone Robotics Motor Control File

void initMotors(){
  motors[0].attach(ESC1);
  motors[1].attach(ESC2);
  motors[2].attach(ESC3);
  motors[3].attach(ESC4);
}

void setThrottle(double motorThrottle, int motorID) {
  int pulseWidth;
  if(motorThrottle == -1)
    pulseWidth = 900;
  else
    pulseWidth = round((motorThrottle/100)*THROTTLE_RANGE+ZERO_THROTTLE);

  motors[motorID].write(pulseWidth);
}

void setThrottleAll(double motorThrottle) {
  for(int i=0; i<4; i++) {
    setThrottle(motorThrottle,i);
  }
}

void updateMotors(){
  setThrottle(motor0Out, 0);
  setThrottle(motor1Out, 1);
  setThrottle(motor2Out, 2);
  setThrottle(motor3Out, 3);
}


