// SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
// SPDX-License-Identifier: MPL-2.0

#include "Arduino_RouterBridge.h"

void setup() {
    // Start the connection to the Monitor tab
    Monitor.begin();
}

void loop() {
    // Print the message
    Monitor.println("Hello im arduino");
    
    // Wait 1 second
    delay(1000);
}
