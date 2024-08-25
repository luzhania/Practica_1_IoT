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

class Led
{
private:
  unsigned int pin;
  unsigned char state;

public:
  Led() : pin(0), state(LOW)
  {
  }

  Led(unsigned int pin)
  {
    this->pin = pin;
    pinMode(pin, OUTPUT);
    state = LOW;
  }

  void turnOn()
  {
    digitalWrite(pin, HIGH);
    state = HIGH;
  }

  void turnOff()
  {
    digitalWrite(pin, LOW);
    state = LOW;
  }
};

#define LED_QUANTITY 8

class LedController
{
private:
  Led usedLeds[LED_QUANTITY];
  unsigned int lastLed = 0;
  const unsigned int stride = 6;

public:
  LedController()
  {
  }

  void addLed(unsigned int pin)
  {
    usedLeds[lastLed] = Led(pin);
    lastLed++;
  }

  void updateLeds(unsigned long actualDistance)
  {
    for (int led = 0; led <= lastLed; led++)
    {
      unsigned long fixedDistance = (led + 1) * stride;
      controlLed(usedLeds[led], actualDistance, fixedDistance);
    }
  }

  void controlLed(Led &led, unsigned long actualDistance, unsigned long fixedDistance)
  {
    if (isInRange(actualDistance, fixedDistance))
      led.turnOn();
    else
      led.turnOff();
  }

  bool isInRange(unsigned long actualDistance, unsigned long fixedDistance)
  {
    return actualDistance < fixedDistance;
  }
};

UltrasonicSensor ultrasonicSensor(2, 5); // Trigger on pin 10, echo on pin 11
LedController ledController;

void setup()
{
  Serial.begin(9600);
  ledController.addLed(13);
  ledController.addLed(14);
  ledController.addLed(26);
  ledController.addLed(33);
}

void loop()
{
  unsigned int cm = ultrasonicSensor.getDistanceInCM();
  Utilities::serialPrintNonBlockingDelay(100, cm);
  ledController.updateLeds(cm);
}