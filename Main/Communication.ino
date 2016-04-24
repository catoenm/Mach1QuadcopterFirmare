
//Cyclone Robotics Communication File

void updateComm(){
  if (rf69.available())
  {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {

      userMessage = buf[0]<<8|buf[1];
      attitudePid = buf[2];
      pidValue = buf[3];
      
      pidValues[attitudePid] = (double)(pidValue * 2.0)/200.0;

      Serial.println(userMessage/10.24);
      
      for (int i = 0; i < 9; i++){
        Serial.print(pidValues [i]);
        Serial.print(", ");
      }
      Serial.println("");

      throttle = userMessage/10.23;

      rollPid.SetTunings(pidValues[0], pidValues[1], pidValues[2]);
      pitchPid.SetTunings(pidValues[3], pidValues[4], pidValues[5]);
      yawPid.SetTunings(pidValues[6], pidValues[7], pidValues[8]);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}

void initComm(){
  
  userMessage = 0;
  
  
  if (!rf69.init())
    Serial.println("init failed");
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  Serial.println("Init Completed");

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
#if 0
  // For compat with RFM69 Struct_send
  rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  rf69.setPreambleLength(3);
  uint8_t syncwords[] = { 0x2d, 0x64 };
  rf69.setSyncWords(syncwords, sizeof(syncwords));
  rf69.setEncryptionKey((uint8_t*)"thisIsEncryptKey");
#endif
}


