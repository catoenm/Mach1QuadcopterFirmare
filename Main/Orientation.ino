
//Cyclone Robotics Orientation File

void updateOrientation(){
  imu.readIMU();
  
  rollInput = imu.getRoll();
  pitchInput = imu.getPitch();
  yawInput = imu.getYaw();

  Serial.print(F("Orientation: "));
        Serial.print(pitchInput);
        Serial.print(F(" "));
        Serial.print(rollInput);
        Serial.print(F(" "));
        Serial.print(yawInput);
        Serial.println(F(""));
}
void initIMU(){
  imu.enable();
}
