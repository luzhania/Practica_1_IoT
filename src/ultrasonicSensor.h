#include "utilities.h"
#pragma once

class UltrasonicSensor
{
private:
  unsigned int triggerPin;
  unsigned int echoPin;

public:
  UltrasonicSensor(unsigned int trigger, unsigned int echo)
  {
    triggerPin = trigger;
    echoPin = echo;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  unsigned long readDistance()
  {
    digitalWrite(triggerPin, LOW);
    Utilities::nonBlockingDelayFor(2);
    digitalWrite(triggerPin, HIGH);
    Utilities::nonBlockingDelayFor(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echoPin, HIGH);
  }

  unsigned long getDistanceInCM()
  {
    return 0.01723 * readDistance();
  }

  unsigned long getDistanceInInches()
  {
    return getDistanceInCM() / 2.54;
  }
};