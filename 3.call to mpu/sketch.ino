#include <Arduino_RouterBridge.h>

void setup() {
//delay(5000);
  // put your setup code here, to run once:
Bridge.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
Bridge.call("key_word","Hi This message sent from MCU");
delay(3000);
}

