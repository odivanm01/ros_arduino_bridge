/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A 3  //pin 3
  #define LEFT_ENC_PIN_B 18  //pin 18
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A 2  //pin 2
  #define RIGHT_ENC_PIN_B 19   //pin 19
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

