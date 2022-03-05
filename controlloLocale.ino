void controlloLocale_loop(){
  static uint32_t nowTime, pastTime;
    nowTime = millis();
    if(nowTime-pastTime > 250){
      for(int i=0; i < N_DEVIATOI; i++){
      if(pcf8574.digitalRead(1 << (i+1)))
        deviatoioPointerVector[i]->motorCommand = false;
      else
        deviatoioPointerVector[i]->motorCommand = true;
      }
      pastTime = nowTime;
    }
}
