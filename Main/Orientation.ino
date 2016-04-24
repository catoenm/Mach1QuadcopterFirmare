
//Cyclone Robotics Orientation File

void updateOrientation(){
  imu.readIMU();
  
  rollInput = imu.getRoll();
  pitchInput = imu.getPitch();
  yawInput = imu.getYaw();
}
void initIMU(){
  imu.enable();
}
