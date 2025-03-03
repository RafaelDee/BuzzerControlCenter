#include <Arduino.h>
#ifndef CLOCK_H
#define CLOCK_H
class Clock
{
public:
  static bool TimePassed(unsigned long &lastTime, unsigned long timerDelay, bool reset = false)
  {
    bool passed = (millis() - lastTime) > timerDelay;
    if (passed && reset)
    {
      lastTime = millis();
    }
    return passed;
  }
};
#endif