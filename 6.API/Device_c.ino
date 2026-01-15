// UPLOAD THIS TO BOARD B (The Device being tested)

void setup() {
  // Initialize the communication port (Must match Board A baud rate)
  Serial1.begin(115200); 
  
  // Initialize USB serial for debugging (optional, to see what's happening on PC)
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // --- LOGIC 1: Handle Incoming Data (Supports 'write' and 'write_read') ---
  if (Serial1.available()) {
    // Read the incoming command from Board A
    String incoming = Serial1.readString();
    
    // Debug output to PC
    Serial.print("Received from Board A: ");
    Serial.println(incoming);

    // TEST CASE A: Response Test (for uart_writeread)
    // If Board A says "PING", we must reply "PONG" immediately
    if (incoming.indexOf("PING") >= 0) {
       Serial1.print("PONG");
    }

    // TEST CASE B: Action Test (for uart_write)
    // If Board A says "LED_ON", we verify by turning on the light
    if (incoming.indexOf("LED_ON") >= 0) {
       digitalWrite(LED_BUILTIN, HIGH);
    }
    if (incoming.indexOf("LED_OFF") >= 0) {
       digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // --- LOGIC 2: Autonomous Broadcast (Supports 'uart_read') ---
  // Send data periodically so Board A has something to read if it checks
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 3000) { // Every 3 seconds
    Serial1.print("Heartbeat: Board B is alive");
    lastTime = millis();
  }
}
