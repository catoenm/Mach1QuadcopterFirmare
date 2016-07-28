
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
    
//    rollPid.Compute();
//    pitchPid.Compute();
//    yawPid.Compute();

    double rollErr = rollSetpoint - rollInput;
    double pitchErr = pitchSetpoint + pitchInput;
    double yawErr = yawSetpoint - yawInput;

    if (rollIntErr < -30)
      rollIntErr = -30;
    if (rollIntErr >  30)
      rollIntErr =  30;
      
    if (pitchIntErr < -30)
      pitchIntErr = -30;
    if (pitchIntErr >  30)
      pitchIntErr =  30;

    rollOutput = rollErr * ROLL_P + rollIntErr * ROLL_I + (rollErr - rollPrevPrevErr) * ROLL_D;
    pitchOutput = pitchErr * PITCH_P + pitchIntErr * PITCH_I + (pitchErr - pitchPrevErr) * PITCH_D;
    //yawOutput = yawErr * YAW_P + yawIntErr * YAW_I;

    rollPrevPrevErr = rollPrevErr;
    rollPrevErr = rollErr;
    pitchPrevErr = pitchErr;
    //yawPrevErr = yawErr;
          
    
  
    if (throttle == 0){
      motor0Out = 0;
      motor1Out = 0;
      motor2Out = 0;
      motor3Out = 0;
    }
    else{
      // Integrate Error
      rollIntErr += rollErr;
      pitchIntErr += pitchErr;
      //yawIntErr += yawErr;

      // Set outputs 
      motor0Out = throttle - rollOutput + pitchOutput + yawOutput;
      motor1Out = throttle + rollOutput + pitchOutput - yawOutput;
      motor2Out = throttle + rollOutput - pitchOutput + yawOutput;
      motor3Out = throttle - rollOutput - pitchOutput - yawOutput;
    }
  }
}
