#include <Arduino.h>
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

class UltrasonicSensor
{
private:
  int triggerPin;
  int echoPin;

public:
  UltrasonicSensor(int trigger, int echo)
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

  int getDistanceInCM()
  {
    return 0.01723 * readDistance();
  }

  int getDistanceInInches()
  {
    return getDistanceInCM() / 2.54;
  }
};

#define LED_QUANTITY 8

class LedController
{
private:
  const unsigned int stride = 6;
  const unsigned int usedPins[LED_QUANTITY] = {2, 3, 4, 5, 6, 7, 8, 9};

public:
  LedController()
  {
    for (int led = 0; led < LED_QUANTITY; led++)
      pinMode(usedPins[led], OUTPUT);
  }

  void updateLeds(unsigned long distance)
  {
    for (int led = 0; led < LED_QUANTITY; led++)
    {
      unsigned long fixedDistance = (led + 1) * stride;
      controlLed(usedPins[led], distance, fixedDistance);
    }
  }

  void controlLed(unsigned int pin, unsigned long actualDistance, unsigned long fixedDistance)
  {
    if (isInRange(actualDistance, fixedDistance))
      digitalWrite(pin, HIGH);
    else
      digitalWrite(pin, LOW);
  }

  bool isInRange(unsigned long actualDistance, unsigned long fixedDistance)
  {
    return actualDistance <= fixedDistance;
  }
};

UltrasonicSensor ultrasonicSensor(10, 11); // Trigger on pin 10, echo on pin 11
LedController ledController;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  unsigned int cm = ultrasonicSensor.getDistanceInCM();
  Utilities::serialPrintNonBlockingDelay(100, cm);
  ledController.updateLeds(cm);
}
