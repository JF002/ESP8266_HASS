#pragma once
#include <atomic>
#include <Arduino.h>
#include <PubSubClient.h>

std::atomic<bool> isMotion;
int pin = D1;

void OnMotion() {
  int val = digitalRead(pin);
  isMotion = (val == HIGH);
}

class MotionSensor {
  public:
    MotionSensor(){}
    void Setup() {
      pinMode(pin, INPUT_PULLUP);
      isMotion = false;
      lastReturnedState = false;
      firstRun = true;
      currentMotion = false;
      attachInterrupt(digitalPinToInterrupt(pin), OnMotion, CHANGE);
    }

    void Loop() {
      currentMotion = isMotion;
    }

    bool IsMotionStateChanged() {
      if(firstRun || lastReturnedState != currentMotion) {
        firstRun = false;
        lastReturnedState = currentMotion;
        return true;
      }

      return false;
    }

    bool IsMotion() {
      return currentMotion;
    }

  private:
    bool firstRun;
    bool lastReturnedState;
    bool currentMotion;
};
