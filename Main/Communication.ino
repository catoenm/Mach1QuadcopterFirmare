
//Cyclone Robotics Communication File

void updateComm(){
  if (rf69.available())
  {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {

      userMessage = buf[0]<<8|buf[1];

      //Serial.println(userMessage/10.23);

      throttle = userMessage/10.23;
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


