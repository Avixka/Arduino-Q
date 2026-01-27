/*
 * PROJECT: ISUNA Modbus Device Simulator (Slave)
 * ROLE: Acts as the DCAC Inverter
 * PROTOCOL: ISUNA V2 (Float Big-Endian, CRC16)
 */

#include <Arduino.h>

// =================================================================================
// 1. SIMULATION STATE
// =================================================================================
namespace SimState {
    const uint8_t SLAVE_ID = 0x01;
    
    // Simulated Registers
    float grid_voltage = 230.50f; //[cite_start]// Addr 0x1000 [cite: 182]
    float max_charge   = 25.00f;  //[cite_start]// Addr 0x3040 [cite: 288]
    
    // Metrics
    unsigned long last_packet = 0;
    bool led_state = false;
}

// =================================================================================
// 2. HELPER FUNCTIONS (CRC & Endianness)
// ================================================================================

// CRC16 (Poly 0xA001) - Matches Master Logic [cite: 555-584]
uint16_t calculateCRC(const uint8_t* buffer, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Convert Float to Big-Endian Bytes (ABCD) [cite: 48-49]
void floatToBytes(float val, uint8_t* dest) {
    union { float f; uint32_t i; } u;
    u.f = val;
    dest[0] = (u.i >> 24) & 0xFF;
    dest[1] = (u.i >> 16) & 0xFF;
    dest[2] = (u.i >> 8) & 0xFF;
    dest[3] = u.i & 0xFF;
}

// Convert Big-Endian Bytes to Float
float bytesToFloat(const uint8_t* src) {
    union { uint32_t i; float f; } u;
    u.i = ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
          ((uint32_t)src[2] << 8)  | ((uint32_t)src[3]);
    return u.f;
}

// =================================================================================
// 3. PACKET PROCESSING
// =================================================================================

// Process Read Request (FC 0x03)
void handleRead(uint8_t* frame) {
    uint16_t start_addr = (frame[2] << 8) | frame[3];
    uint16_t quantity   = (frame[4] << 8) | frame[5];
    
    // We only support reading 2 registers (1 float) at a time for this demo
    if (quantity != 2) return; 

    float value = 0.0f;
    
    // Map Address to Simulated Data
    if (start_addr == 0x1000) value = SimState::grid_voltage;
    else if (start_addr == 0x3040) value = SimState::max_charge;
    else return; // Unknown register, ignore

    // Build Response: Addr(1) + FC(1) + Bytes(1) + Data(4) + CRC(2)
    uint8_t response[9];
    response[0] = SimState::SLAVE_ID;
    response[1] = 0x03;
    response[2] = 0x04; // 4 Bytes payload
    
    floatToBytes(value, &response[3]);
    
    uint16_t crc = calculateCRC(response, 7);
    response[7] = crc & 0xFF;        // CRC Low
    response[8] = (crc >> 8) & 0xFF; // CRC High
    
    // Send with slight delay to simulate processing time
    delay(5);
    Serial.write(response, 9);
}

// Process Write Request (FC 0x10)
void handleWrite(uint8_t* frame) {
    uint16_t start_addr = (frame[2] << 8) | frame[3];
    uint16_t quantity   = (frame[4] << 8) | frame[5];
    uint8_t  byte_count = frame[6];
    
    if (quantity != 2 || byte_count != 4) return;

    // Extract Float value from frame (Data starts at index 7)
    float new_val = bytesToFloat(&frame[7]);
    
    // Update State
    if (start_addr == 0x3040) {
        SimState::max_charge = new_val;
        // Blink LED fast to indicate write
        digitalWrite(LED_BUILTIN, HIGH); delay(50); digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Build Response (Echo): Addr(1)+FC(1)+Start(2)+Qty(2)+CRC(2)
    uint8_t response[8];
    response[0] = SimState::SLAVE_ID;
    response[1] = 0x10;
    response[2] = (start_addr >> 8) & 0xFF;
    response[3] = start_addr & 0xFF;
    response[4] = (quantity >> 8) & 0xFF;
    response[5] = quantity & 0xFF;
    
    uint16_t crc = calculateCRC(response, 6);
    response[6] = crc & 0xFF;
    response[7] = (crc >> 8) & 0xFF;

    delay(5);
    Serial.write(response, 8);
}

// =================================================================================
// 4. MAIN LOOP
// =================================================================================

void setup() {
    // Slave uses Serial port to talk to Master
    // If using UNO R3/R4, this is Pins 0/1.
    Serial.begin(115200); 
    pinMode(LED_BUILTIN, OUTPUT);
}

uint8_t rxBuffer[64];
int rxIndex = 0;
unsigned long lastByteTime = 0;

void loop() {
    // 1. Modbus RTU Receiver (Timeout based)
    if (Serial.available()) {
        rxBuffer[rxIndex++] = Serial.read();
        lastByteTime = millis();
        if (rxIndex >= 64) rxIndex = 0; // Overflow safety
    }

    // 2. Process Packet on Silence (3.5 char times ~ 2ms at 115200)
    // We use 5ms to be safe.
    if (rxIndex > 0 && (millis() - lastByteTime > 5)) {
        
        // A. Basic Validation
        if (rxBuffer[0] == SimState::SLAVE_ID && rxIndex >= 8) {
            
            // B. Verify CRC
            uint16_t receivedCRC = rxBuffer[rxIndex-2] | (rxBuffer[rxIndex-1] << 8); // Packet is Little Endian CRC? 
            // WAIT: Master sends CRC Low first. 
            uint16_t packetCRC = rxBuffer[rxIndex-2] | (rxBuffer[rxIndex-1] << 8);
            // Actually, let's just calc ours and compare bytes
            
            uint16_t calcCRC = calculateCRC(rxBuffer, rxIndex - 2);
            
            if ((calcCRC & 0xFF) == rxBuffer[rxIndex-2] && (calcCRC >> 8) == rxBuffer[rxIndex-1]) {
                // CRC OK! Route Command
                if (rxBuffer[1] == 0x03) handleRead(rxBuffer);
                else if (rxBuffer[1] == 0x10) handleWrite(rxBuffer);
            }
        }
        
        // Reset Buffer
        rxIndex = 0;
    }
}
