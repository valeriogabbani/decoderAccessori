/*
 * CLasse che identifica un evento asociato ad un pin
 */
class switchEvent{
  public:
    switchEvent(int pinNumber);
    bool test();
    void loop();
  private:
    int _pinNumber;
    int lastPinstatus;
    bool armed, eventDetected;
    uint32_t timeMark;
};
