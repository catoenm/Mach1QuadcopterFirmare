
//Cyclone Robotics Orientation File

void updateOrientation(){
  imu.readIMU();
  
  Imu_Roll = imu.getRoll();
  Imu_Pitch = imu.getPitch();
  Imu_Yaw = imu.getYaw();
}
void initIMU(){
  imu.enable();
}
