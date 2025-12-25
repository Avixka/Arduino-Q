/* * MCU SIDE CODE (STM32U585)
 * Acts as the RPC "Server"
 * Updated for Arduino_RouterBridge library
 */

#include <Arduino_RouterBridge.h>

const int ledPin = LED_BUILTIN;

// --- 1. Define Functions ---
// These replace the "if (receivedMsg == ...)" checks.
// Instead of printing the response, we return it.

String cmd_led_on() {
    digitalWrite(ledPin, HIGH);
    // This string is sent back to the Linux MPU automatically
    return "OK_ON"; 
}

String cmd_led_off() {
    digitalWrite(ledPin, LOW);
    // This string is sent back to the Linux MPU automatically
    return "OK_OFF";
}

void setup() {
    // 2. Initialize Hardware
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); // Start OFF

    // 3. Start the Bridge (Replaces RPC.begin())
    Bridge.begin();
    Monitor.begin(); // Optional: Used for debugging via Serial Monitor

    // 4. Register Commands
    // We tell the Bridge: "If you receive 'cmd_led_on', run the function cmd_led_on"
    // We use provide_safe because we are returning Strings.
    
    if (!Bridge.provide_safe("cmd_led_on", cmd_led_on)) {
        Monitor.println("Error registering cmd_led_on");
    }

    if (!Bridge.provide_safe("cmd_led_off", cmd_led_off)) {
        Monitor.println("Error registering cmd_led_off");
    }
    
    Monitor.println("RPC Server Ready");
}

void loop() {
    // 5. Loop is Empty!
    // Unlike RPC.h, you do NOT need to check 'available()' or parse strings here.
    // The library handles all incoming messages in the background automatically.
    
    // If you wanted to *send* a message to the MPU (like a sensor value), 
    // you would do it here using Bridge.notify() or Bridge.call().
}
