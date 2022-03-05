//#define ENABLE_DEBUG_OUTPUT
#include <Adafruit_PWMServoDriver.h>
#define MAX_PWM 380
#define MIN_PWM 100
#define MAX_NORMAL_LIMIT  70
#define MAX_REVERSE_LIMIT  110
#define SERVO_TIMOUT    5000
#define NORMALE false
#define ROVESCIO true
/*
 *  Classe CONTENITORE "deviatoio" AGGREGA Adafruit_PWMServoDriver
 */
 
class deviatoio  {
  
  public:
  deviatoio(int controlPin, Adafruit_PWMServoDriver *servoDriverPointer); //costruttore 
  void setNormalPositionLimit(int limit);
  void setReversePositionLimit(int limit);
  void disable();
  int  readReversePositionLimit();
  int  readNormalPositionLimit();
  bool normalPosition, reversePosition;
  void motore();
  bool motorCommand; //false = normale; true = rovescio
  void setToangle( int angle);
  
  private:
  
  int normalPositionLimit;
  int reversePositionLimit;
  bool motorClocksignal;
  int motorCounter;
  int motorStatus;
  uint32_t motorClocktimer;
  int _controlPin;
  int _comando;
  int angleTopwm(int angle);
  uint32_t timer;
  Adafruit_PWMServoDriver *_servoDriverPointer;  

};
