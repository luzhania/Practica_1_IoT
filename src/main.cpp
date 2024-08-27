#include <Arduino.h>
#include "ultrasonicSensor.h"
#include "ledController.h"

LedController ledController;
UltrasonicSensor ultrasonicSensor(19, 22);

void setup()
{
  Serial.begin(9600);
  ledController.addLed(13);
  ledController.addLed(14);
  ledController.addLed(26);
  ledController.addLed(33);
  ledController.addLed(4);
  ledController.addLed(5);
  ledController.addLed(21);
}

void loop()
{
  unsigned long cm = ultrasonicSensor.getDistanceInCM();
  Utilities::serialPrintNonBlockingDelay(100, cm);
  ledController.updateLeds(cm);
}