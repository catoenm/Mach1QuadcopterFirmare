
//Cyclone Robotics PID Control File

void initPID(){
  rollPid.SetMode(AUTOMATIC);
  pitchPid.SetMode(AUTOMATIC);
  yawPid.SetMode(AUTOMATIC);
}

void updatePID(){
  if(motorState == spinning){
    rollSetpoint = 0;
    pitchSetpoint = 0;
    yawSetpoint = 0;
    
    rollPid.Compute();
    pitchPid.Compute();
    yawPid.Compute();
  
    if (throttle == 0){
      motor0Out = 0;
      motor1Out = 0;
      motor2Out = 0;
      motor3Out = 0;
    }
    else{
      motor0Out = throttle + rollOutput - pitchOutput + yawOutput;
      motor1Out = throttle - rollOutput + pitchOutput - yawOutput;
      motor2Out = throttle - rollOutput + pitchOutput + yawOutput;
      motor3Out = throttle + rollOutput - pitchOutput - yawOutput;
    }
  }
}
