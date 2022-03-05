/* DA CAMBIARE DI POSTO */////
  switchEvent::switchEvent(int pinNumber){
      _pinNumber = pinNumber;
      pinMode(_pinNumber, INPUT_PULLUP);
      armed = eventDetected = false;
      lastPinstatus = digitalRead(_pinNumber);
  }
  void switchEvent::loop(){
    //ground state
    if ( (digitalRead(_pinNumber) == 0) && (lastPinstatus == 1) && !armed && !eventDetected){
        eventDetected = true;
        timeMark = millis();
    }
    if (eventDetected){
      if(millis() - timeMark > 100) {
        armed = true;
        eventDetected = false;
      }
    }   
  lastPinstatus = digitalRead(_pinNumber);
  }

  bool switchEvent::test(){
    if ( armed ) {
      armed = false;
      return true;
    }
    return false;  
  }
/* DA CAMBIARE DI POSTO */////
