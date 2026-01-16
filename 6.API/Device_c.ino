#include <Arduino_RouterBridge.h>

void setup() {
  Serial.begin(115200); 
  Monitor.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {   
    int firstByte = Serial.peek();

    if (firstByte == 0 || firstByte == 1) {
        int command = Serial.read();
        
        digitalWrite(LED_BUILTIN, command);
        
        if (command == 1) {
             Serial.print("LED_Turned_OFF");
        } else {
             Serial.print("LED_Turned_ON");
        }
    }
    
    else {
        String message = Serial.readString();
        Monitor.println(message); 
        
        Serial.print("Hi from -> Device");
    }
  }
}
