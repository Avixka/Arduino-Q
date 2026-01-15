#include <Arduino_RouterBridge.h>

void setup() {
  Bridge.begin();
  Serial1.begin(115200); 
  Bridge.provide("uart_write", uartWrite);
  Bridge.provide("uart_read", uartRead);
  Bridge.provide("uart_writeread", uartWriteRead); 
}

void loop() {
 
}

void uartWrite(String data) {
  Serial1.print(data);
}

String uartRead(String ignored) {
  if (Serial1.available()) {
    return Serial1.readString();
  }
  return "Nothing recived";
}

String uartWriteRead(String command) {
  while(Serial1.available()) { Serial1.read(); }

  Serial1.print(command);

  unsigned long startTime = millis();
  while (Serial1.available() == 0) {
    if (millis() - startTime > 1000) {
      return "ERROR: TIMEOUT";
    }
  }
  
  String response = Serial1.readString();
  return response;
}
