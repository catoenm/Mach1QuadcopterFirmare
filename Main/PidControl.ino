
//Cyclone Robotics PID Control File

void initPID(){
  
}

void updatePID(){
  rollOutput = 0;
  pitchOutput = 0;
  yawOutput = 0;

  motor0Out = throttle;
  motor1Out = throttle;
  motor2Out = throttle;
  motor3Out = throttle;
  
//  int PIDroll_val= (int)PIDroll.Compute((float)setX-gy_aver);
//  int PIDpitch_val= (int)PIDpitch.Compute((float)setY-gx_aver);
//  int PIDyaw_val= (int)PIDyaw.Compute((float)setZ-gz_aver);
//
//  int m0_val=throttle+PIDroll_val+PIDpitch_val+PIDyaw_val;
//  int m1_val=throttle-PIDroll_val+PIDpitch_val-PIDyaw_val;
//  int m2_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
//  int m3_val=throttle-PIDroll_val-PIDpitch_val+PIDyaw_val;
}
