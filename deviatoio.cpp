/*
 * Metodi per la classe
 */

#include "deviatoio.h"

//Costruttore di classe 
   deviatoio::deviatoio (int controlPin, Adafruit_PWMServoDriver *servoDriverPointer ){
   _controlPin = controlPin;
   _servoDriverPointer = servoDriverPointer;
   normalPositionLimit = reversePositionLimit = 90;
   motorCounter = normalPositionLimit ;
   motorCommand = false;
   motorStatus = 10;
   setToangle(90);//Posiziona i servo a RIPOSO.
   //setNormalPosition();
}

//Spengo il motore mettondo il PWM a 0 (no signal)
void deviatoio::disable(){
  _servoDriverPointer->setPin(_controlPin, 0, true);
}

/*
 * FSM che siula il funzionamento del motore con finecorsa 
 * Il segnale di comando è la variabile motorCommand
 * Simula il movimento lento del motore
 */
void deviatoio::motore() {
  if (millis() - motorClocktimer > 50){
    motorClocktimer = millis();
    motorClocksignal = true;
  }
  else motorClocksignal = false;
  //Stato iniziale ?????
  switch (motorStatus){
    //Stato iniziale del motore
    case 10:
      motorCounter = reversePositionLimit;
      motorStatus = 3;
      break;
    //stato stabile normale
    case 0:
      if (motorCommand == ROVESCIO) motorStatus = 2;
      disable();
      normalPosition = true;
      reversePosition = false;
      break;
    //stato stabile rovescio
    case 1:
      if (motorCommand == NORMALE) motorStatus = 3;
      disable();
      normalPosition = false;
      reversePosition = true;
      break;
      
    //transizione normale->rovescio
    //Il clock comanda il movimento
    case 2:
      normalPosition = false;
      reversePosition = false;
      if(motorClocksignal){
        if (normalPositionLimit > reversePositionLimit){
          motorCounter = max(motorCounter - 1, reversePositionLimit);
        }
        if (normalPositionLimit < reversePositionLimit){
          motorCounter = min(motorCounter + 1, reversePositionLimit);
        }
        Serial.println(motorCounter);
        setToangle(motorCounter);
      }
      if (abs(motorCounter - reversePositionLimit) == 0){
        motorStatus = 1;
      }
      break;

    //transizione rovescio->normale
    case 3:
      normalPosition = false;
      reversePosition = false;
      if(motorClocksignal){
        if (normalPositionLimit > reversePositionLimit){
          motorCounter = min(motorCounter + 1, normalPositionLimit);
        }
        if (normalPositionLimit < reversePositionLimit){
          motorCounter = max(motorCounter - 1, normalPositionLimit);
        }
        Serial.println(motorCounter);
        setToangle(motorCounter);
      }
      if (abs(motorCounter - normalPositionLimit) == 0){
        motorStatus = 0;
      }
      break;
      
     default:
        motorStatus = 0;
  }//switch
}
/*
 * Imposta il finecorsa norale
 */
void deviatoio::setNormalPositionLimit(int limit){
  normalPositionLimit = limit;
}
/*
 * Imposta il finecorsa rovescio
 */
void deviatoio::setReversePositionLimit(int limit){
  reversePositionLimit = limit; 
}

/*
 * Legge il valore (angolo) del finecorsa normale
 */
int deviatoio::readNormalPositionLimit(){
  return normalPositionLimit;
}
/*
 * Legge il valore (angolo) del finecorsa rovescio
 */
int deviatoio::readReversePositionLimit(){
  return reversePositionLimit;
}
/*
 * Converte il valore dell'angolo di rotazione in Duty del PWM servo
 * Con finecorsa di sicurezza:
 */
int deviatoio::angleTopwm(int angle){
  int pwm = map(angle, 0, 180, MIN_PWM, MAX_PWM);
  if (pwm > MAX_PWM) return MAX_PWM;
  if (pwm < MIN_PWM) return MIN_PWM;
  return pwm;
}
/*
 * Imposta la posizione del controllo aghi in gradi servo)
 */
void deviatoio::setToangle(int angle){
  int pwm = angleTopwm(angle);
  _servoDriverPointer->setPin(_controlPin, pwm, false);
}
