
//Cyclone Robotics PID Control File

void initPID() {
  rollPid.SetMode(AUTOMATIC);
  pitchPid.SetMode(AUTOMATIC);
  yawPid.SetMode(AUTOMATIC);
}

void updatePID() {
  if (motorState == spinning) {
    rollSetpoint = 0;
    pitchSetpoint = 0;
    yawSetpoint = 0;

    rollPid.Compute();
    pitchPid.Compute();
    yawPid.Compute();

//    if (throttle == 0) {
//      motor0Out = 0;
//      motor1Out = 0;
//      motor2Out = 0;
//      motor3Out = 0;
//    }
//    else {
      //      motor0Out = throttle - rollOutput - pitchOutput + yawOutput;
      //      motor1Out = throttle + rollOutput + pitchOutput - yawOutput;
      //      motor2Out = throttle + rollOutput + pitchOutput + yawOutput;
      //      motor3Out = throttle - rollOutput - pitchOutput - yawOutput;
      switch (motorCycle) {
        case mot1:
          if (digitalRead(BUTTON))
            buttonHit = 1;
          if (!(digitalRead(BUTTON)) && buttonHit) {
            motorCycle = mot2;
            buttonHit = 0;
          }
          motor1Out = 30;
          motor2Out = 0;
          motor3Out = 0;
          motor0Out = 0;
          break;
        case mot2:
          if (digitalRead(BUTTON))
            buttonHit = 1;
          if (!(digitalRead(BUTTON)) && buttonHit) {
            motorCycle = mot3;
            buttonHit = 0;
          }
          motor1Out = 0;
          motor2Out = 30;
          motor3Out = 0;
          motor0Out = 0;
          break;
        case mot3:
          if (digitalRead(BUTTON))
            buttonHit = 1;
          if (!(digitalRead(BUTTON)) && buttonHit) {
            motorCycle = mot4;
            buttonHit = 0;
          }
          motor1Out = 0;
          motor2Out = 0;
          motor3Out = 30;
          motor0Out = 0;
          break;
        case mot4:
          if (digitalRead(BUTTON))
            buttonHit = 1;
          if (!(digitalRead(BUTTON)) && buttonHit) {
            motorCycle = mot1;
            buttonHit = 0;
          }
          motor1Out = 0;
          motor2Out = 0;
          motor3Out = 0;
          motor0Out = 30;
          break;
      }
    //}
  }
}
