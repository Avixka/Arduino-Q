#include "Arduino_RouterBridge.h"

void setup() {
  Bridge.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);

  Bridge.provide("key_word", blink_function);
}

void loop() {
  
}

void blink_function(bool state) {
  digitalWrite(LED_BUILTIN, state);
}
