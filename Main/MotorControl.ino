void initMotors(){
  motors[0].attach(ESC1);
  motors[1].attach(ESC2);
  motors[2].attach(ESC3);
  motors[3].attach(ESC4);
}

void setThrottle(double throttle, int motorID) {
  int pulseWidth;
  if(throttle == -1)
    pulseWidth = 900;
  else
    pulseWidth = round((throttle/100)*THROTTLE_RANGE+ZERO_THROTTLE);

  motors[motorID].write(pulseWidth);
}

void setThrottleAll(double throttle) {
  for(int i=0; i<4; i++) {
    setThrottle(throttle,i);
  }
}

void updateMotors(){
  switch(motorState) {
    
    case idle:
      if(digitalRead(BUTTON)) {
        while(digitalRead(BUTTON));
        motorState = startSpinning;
      }
      break;
      
    case startSpinning:
      digitalWrite(RED, 0);
      delay(200);
      for (int i=0; i<3; i++) {
        playTone(700, 500);
        digitalWrite(RED, 0);
        delay(500);
      }
    
      playTone(500, 1200);
      digitalWrite(RED, 0);
      delay(500);

      digitalWrite(GREEN, 1);

      if(sweep) {
        setThrottleAll(0);
      }
      else {
        setThrottleAll(maxThrottle);
      }
        
      spinTimeStamp = millis();
      motorState = spinning;
      break;
      
    case spinning: 
      int elapsedTime = millis()-spinTimeStamp;
      if(elapsedTime > spinTime) {
        setThrottleAll(-1);
        motorState = idle;
      }
      else if(sweep) {
        setThrottleAll((elapsedTime/spinTime)*maxThrottle);
      }
      break;
  }
}


