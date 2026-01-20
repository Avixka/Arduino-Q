#include "Arduino_RouterBridge.h"

void setup() {
  Bridge.begin();
  Bridge.provide("blink_led", blink_function);
  
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  
  Bridge.call("print_msg", "Hi, this message sent from MCU");
  delay(3000);

}

void blink_function(bool state) {
  digitalWrite(LED_BUILTIN, state);
}
