#include "led.h"

led::led(int pinNumber){
  ledPin = pinNumber;
  pinMode(ledPin, OUTPUT);
  negativeLogic = false;
  _onTime = 1000;
  _offTime = 1000;
  _blinkTime = 500;
  off();
}

led::led(int pinNumber, bool logic){
  ledPin = pinNumber;
  pinMode(ledPin, OUTPUT);
  negativeLogic = logic;
  _onTime = 1000;
  _offTime = 1000;
  _blinkTime = 500;
  off();
}

void led::loop(){
  nowTime = millis();
  if(blinkControl){
    if (nowTime - lastTime > _blinkTime){
      toggle();
      lastTime = nowTime;
    }
  }
  else off();
}

void led::on(){
  blinkControl = false;
  if(negativeLogic)  digitalWrite(ledPin, LOW);
  else digitalWrite(ledPin, HIGH);
}

void led::off(){
  blinkControl = false;
  if(negativeLogic)  digitalWrite(ledPin, HIGH);
  else digitalWrite(ledPin, LOW);
}

void led::toggle(){
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void led::blink(){
    blinkControl = true;
}
