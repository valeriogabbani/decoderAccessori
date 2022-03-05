/*
 * Classe per segnalatore a LED
 */
#include <Arduino.h>
class led{

  public:
    led(int ledPin);
    led(int ledPin, bool logic);
    void loop();
    void on();
    void off();
    void toggle();
    void blink();
  
  private:
    bool negativeLogic;
    bool blinkControl;
    int ledPin;
    uint32_t lastTime, nowTime;
    int _onTime, _offTime, _blinkTime;
  
};
