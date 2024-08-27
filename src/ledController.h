#include "led.h"
#pragma once
#define LED_QUANTITY 8

class LedController
{
private:
  Led usedLeds[LED_QUANTITY];
  unsigned long distanceToObject = 0;
  const unsigned int STRIDE = 6;
  unsigned int lastLed = 0;

public:
  LedController()
  {
    lastLed = 0;
  }

  void addLed(unsigned int pin)
  {
    if (lastLed < LED_QUANTITY)
    {
      usedLeds[lastLed] = Led(pin);
      lastLed++;
    }
  }

  void updateLeds(unsigned long distanceToObject)
  {
    this->distanceToObject = distanceToObject;
    for (int ledPosition = 0; ledPosition <= lastLed; ledPosition++)
      controlLed(ledPosition);
  }

  void controlLed(unsigned int ledPosition)
  {
    if (isInRange(ledPosition))
      usedLeds[ledPosition].turnOn();
    else
      usedLeds[ledPosition].turnOff();
  }

  bool isInRange(unsigned int ledPosition)
  {
    return distanceToObject <= getFixedDistance(ledPosition);
  }

  unsigned int getFixedDistance(unsigned int ledPosition)
  {
    return (ledPosition + 1) * STRIDE;
  }
};