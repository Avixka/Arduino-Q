#include <Arduino_RouterBridge.h>

void setup() {
  Bridge.begin();
  Serial.begin(115200); 
  
  Serial.setTimeout(2000); 

  Bridge.provide("uart_write", uartWrite);
  Bridge.provide("uart_read", uartRead);
  Bridge.provide("uart_writeread", uartWriteRead); 
}

void loop() {
  
}

void uartWrite(String data) {
  while(Serial.available()) { Serial.read(); }
  Serial.print(data);
}

String uartRead(String ignored) {
  unsigned long startTime = millis();
  while (Serial.available() == 0) {
    if (millis() - startTime > 2000) {
      return "ERROR: TIMEOUT";
    }
  }
  return Serial.readString();
}

String uartWriteRead(String command) {
  while(Serial.available()) { Serial.read(); }

  Serial.print(command);

  unsigned long startTime = millis();
  while (Serial.available() == 0) {
    if (millis() - startTime > 2000) {
      return "ERROR: TIMEOUT";
    }
  }
  
  return Serial.readString();
}
