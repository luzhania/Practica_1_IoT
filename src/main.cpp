#include <Arduino.h>

class UltrasonicSensor {
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
	

  	void nonBlockingDelayFor(unsigned long microseconds) {
      unsigned long startTime = micros();
      while (micros() - startTime < microseconds);
    }
  
    unsigned long readDistance() {
      digitalWrite(triggerPin, LOW);
      nonBlockingDelayFor(2);
      digitalWrite(triggerPin, HIGH);
      nonBlockingDelayFor(10);
      digitalWrite(triggerPin, LOW);
      return pulseIn(echoPin, HIGH);  
    }

    int getDistanceInCM() {
      return 0.01723 * readDistance();
    }

    int getDistanceInInches() {
      return getDistanceInCM() / 2.54;
    }
};

#define LED_QUANTITY 8

class LedController {
  private:
  	const unsigned int stride = 6;
  	const unsigned int usedPins[LED_QUANTITY]={2,3,4,5,6,7,8,9};
  	
  public:
  	LedController() {
      for(int led = 0; led < LED_QUANTITY; led++){
      	pinMode(usedPins[led], OUTPUT);
      }
    }
    
    void controlLed( unsigned int pin, unsigned long actualDistance, unsigned long fixedDistance) {
      if (actualDistance <= fixedDistance) {
        digitalWrite(pin, HIGH);
      } else {
        digitalWrite(pin, LOW);
      }
    }

    void updateLeds(unsigned long distance) {
      for (int led = 0; led < LED_QUANTITY; led++) {
        unsigned long fixedDistance = (led+1) * stride;
        controlLed(usedPins[led], distance, fixedDistance);
      }
    }
};

UltrasonicSensor sensor(10, 11);  // Trigger on pin 10, echo on pin 11
LedController ledController;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int cm = sensor.getDistanceInCM();
  static unsigned long lastMeasurement = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMeasurement >= 100) {
    Serial.print(cm);
    Serial.println("cm");
    lastMeasurement = currentMillis;
  }
  ledController.updateLeds(cm);
}
