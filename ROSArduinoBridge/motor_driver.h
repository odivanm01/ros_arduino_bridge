/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef MAXON_MOTOR_DRIVER
  #define RIGHT_MOTOR_MOVE 5
  #define RIGHT_MOTOR_ENABLE 6
  #define RIGHT_MOTOR_DIRECTION 7
  
  #define LEFT_MOTOR_MOVE   8
  #define LEFT_MOTOR_ENABLE 9
  #define LEFT_MOTOR_DIRECTION 10

  #define RIGHT_SWEEPER_MOVE 3 // M2 motor
  #define RIGHT_SWEEPER_DIRECTION 11 
  
  #define LEFT_SWEEPER_MOVE 4 // M1 motor
  #define LEFT_SWEEPER_DIRECTION 2

  #define 
  #define
#endif

bool motor_blocked

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
