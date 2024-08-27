#include <Arduino.h>
#pragma once

class Utilities
{
public:
  static void nonBlockingDelayFor(unsigned long microseconds)
  {
    unsigned long startTime = micros();
    while (micros() - startTime < microseconds)
      ;
  }

  static void serialPrintNonBlockingDelay(unsigned long milliseconds, unsigned int cm)
  {
    static unsigned long lastMeasurement = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastMeasurement >= 100)
    {
      Serial.print(cm);
      Serial.println("cm");
      lastMeasurement = currentMillis;
    }
  }
};