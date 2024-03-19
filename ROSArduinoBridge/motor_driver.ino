/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef MAXON_MOTOR_DRIVER

  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    { 
      spd = -spd;
      reverse = 1;
    }
    if (spd > 229.5)
      spd = 229.5;
    else if (spd < 25.5) {
      digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
      digitalWrite(LEFT_MOTOR_ENABLE, LOW); 
      return; }
    
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
    
    if (i == LEFT) { 
      if      (reverse == 0) { digitalWrite(LEFT_MOTOR_DIRECTION, HIGH);}
      else if (reverse == 1) { digitalWrite(LEFT_MOTOR_DIRECTION, LOW);}
        analogWrite(LEFT_MOTOR_MOVE, spd);
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { digitalWrite(RIGHT_MOTOR_DIRECTION, LOW);}
      else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_DIRECTION, HIGH);}
      analogWrite(RIGHT_MOTOR_MOVE, spd);
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
