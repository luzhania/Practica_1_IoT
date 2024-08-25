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

  unsigned int getDistanceInCM()
  {
    return 0.01723 * readDistance();
  }

  unsigned int getDistanceInInches()
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
  unsigned long distanceToObject = 0;
  const unsigned int stride = 6;
  unsigned int lastLed = 0;

public:
  LedController()
  {
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
    return distanceToObject < getFixedDistance(ledPosition);
  }

  unsigned int getFixedDistance(unsigned int ledPosition)
  {
    return (ledPosition + 1) * stride;
  }
};

LedController ledController;
UltrasonicSensor ultrasonicSensor(2, 5);

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
  unsigned long cm = ultrasonicSensor.getDistanceInCM();
  Utilities::serialPrintNonBlockingDelay(100, cm);
  ledController.updateLeds(cm);
}