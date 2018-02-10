#include "HID-Project.h"

const int pinLed = LED_BUILTIN;

void setup() {
  pinMode(pinLed, OUTPUT);

  Serial1.begin(2000000);

  Gamepad.begin();
}

void loop() {
    digitalWrite(pinLed, HIGH);
    
    Gamepad.xAxis(((int)Serial1.read()) * (1<<7));
    Gamepad.write();
    
    digitalWrite(pinLed, LOW);
    delay(100);
}
