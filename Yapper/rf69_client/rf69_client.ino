#include <SPI.h>
#include <RH_RF69.h>

#define THROTTLE A1
#define PIDCONSTANT A2
#define BUTTON A3

// Singleton instance of the radio driver
RH_RF69 rf69(4);
//RH_RF69 rf69(15, 16); // For RF69 on PJRC breakout board with Teensy 3.1

enum ButtonState {
  pressed,
  released
};

uint8_t pidState = 0;
ButtonState buttonState;

void setup() 
{
  Serial.begin(9600);
  if (!rf69.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  // No encryption
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  // If you are using a high power RF69, you *must* set a Tx power in the
  // range 14 to 20 like this:
  rf69.setTxPower(20);
  buttonState = released;
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}


void loop()
{
  // Button is active low
  switch (buttonState) {
    case pressed:
      if (digitalRead(BUTTON)) {
        pidState++;
        pidState %= 9;
        buttonState = released;
      }
      break;
    case released:
      if (!digitalRead(BUTTON)) {
        buttonState = pressed;
      }
      break;
  }
  
  // Send a message to rf69_server
  uint8_t data[4];
  int throttle = analogRead(THROTTLE);
  int newPidConstant = 1023 - analogRead(PIDCONSTANT);
  data[0] = (uint8_t)(throttle>>8);
  data[1] = (uint8_t)(throttle%256);
  data[2] = pidState;
  data[3] = (uint8_t)(newPidConstant*(200/1023.0));
  
  Serial.print("Throttle: ");
  Serial.println(throttle*(100/1023.0));
  Serial.print("PID State: ");
  Serial.println(pidState);
  Serial.print("PID Constant: ");
  Serial.println((newPidConstant*(255/1023.0)*(2/255.0)));
  Serial.println();
  
  rf69.send(data, sizeof(data));
  
  rf69.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    //Serial.println("No reply, is rf69_server running?");
  }
  delay(400);
}

