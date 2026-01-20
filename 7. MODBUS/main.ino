/*
 * PROJECT: ISUNA Modbus Bridge (MCU Side) - v7.0 (Cleaned)
 * TARGET: Arduino UNO Q (2025)
 * ARCHITECTURE: Transactional Modbus Driver with RPC
 */

#include <Arduino.h>
#include <Arduino_RouterBridge.h> 
#include <vector>

// NOTE: The library creates the 'Bridge' object automatically.
// We do not need to instantiate it manually.

// =================================================================================
// 1. MODBUS DRIVER LAYER
// =================================================================================
namespace IsunaDriver {
    constexpr uint32_t BAUDRATE = 115200;
    
    // CRC16 Calculation (Poly 0xA001)
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

    // Helper: Float to Big-Endian Bytes
    void floatToBytes(float val, uint8_t* dest) {
        union { float f; uint32_t i; } u;
        u.f = val;
        dest[0] = (u.i >> 24) & 0xFF;
        dest[1] = (u.i >> 16) & 0xFF;
        dest[2] = (u.i >> 8) & 0xFF;
        dest[3] = u.i & 0xFF;
    }

    // Helper: Big-Endian Bytes to Float
    float bytesToFloat(const uint8_t* src) {
        union { uint32_t i; float f; } u;
        u.i = ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
              ((uint32_t)src[2] << 8)  | ((uint32_t)src[3]);
        return u.f;
    }

    // Transaction Handler
    int sendRawTransaction(uint8_t* frame, size_t len, uint8_t* response, size_t maxResp) {
        // Critical: Flush old data from buffer
        while (Monitor.available()) Monitor.read();

        Monitor.write(frame, len);

        // Blocking wait for response
        uint32_t start = millis();
        size_t idx = 0;
        
        // Wait loop (200ms timeout)
        while (millis() - start < 200) { 
            if (Monitor.available()) {
                if (idx < maxResp) {
                    response[idx++] = Monitor.read();
                }
            }
        }
        
        return idx; 
    }
}

// =================================================================================
// 2. RPC API LAYER (Exposed Functions)
// =================================================================================

// READ FLOAT (Function Code 03)
float rpc_read_float(int slave_id, int address) {
    uint8_t tx[8];
    uint8_t rx[32]; 

    // Build Request
    tx[0] = (uint8_t)slave_id;
    tx[1] = (uint8_t)0x03; 
    tx[2] = (address >> 8) & 0xFF;
    tx[3] = address & 0xFF;
    tx[4] = 0x00; tx[5] = 0x02; // 2 Registers

    uint16_t crc = IsunaDriver::calculateCRC(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    int received = IsunaDriver::sendRawTransaction(tx, 8, rx, 32);

    if (received < 9) return -9999.0f; // Timeout
    
    // CRC Check
    uint16_t rxCrc = IsunaDriver::calculateCRC(rx, received - 2);
    // Note: Low Byte First for CRC
    if (rx[received-2] != (rxCrc & 0xFF) || rx[received-1] != (rxCrc >> 8)) {
        return -9998.0f; // CRC Error
    }

    return IsunaDriver::bytesToFloat(&rx[3]);
}

// WRITE FLOAT (Function Code 10)
int rpc_write_float(int slave_id, int address, float value) {
    uint8_t tx[13];
    uint8_t rx[8];

    // Build Request
    tx[0] = (uint8_t)slave_id;
    tx[1] = (uint8_t)0x10;
    tx[2] = (address >> 8) & 0xFF;
    tx[3] = address & 0xFF;
    tx[4] = 0x00; tx[5] = 0x02; // 2 Registers
    tx[6] = 0x04; // 4 Bytes
    
    IsunaDriver::floatToBytes(value, &tx[7]);

    uint16_t crc = IsunaDriver::calculateCRC(tx, 11);
    tx[11] = crc & 0xFF;
    tx[12] = (crc >> 8) & 0xFF;

    int received = IsunaDriver::sendRawTransaction(tx, 13, rx, 8);

    if (received < 8) return 0; // Error
    if (rx[1] == 0x10) return 1; // Success
    return 0;
}

// READ AND WRITE (Atomic)
// Performs a write, waits 20ms, then performs a read.
float rpc_read_write(int slave_id, int write_addr, float write_val, int read_addr) {
    int writeSuccess = rpc_write_float(slave_id, write_addr, write_val);
    
    if (!writeSuccess) return -9997.0f; // Write failed, abort read

    delay(20); // Small safety buffer to let Slave process the write
    return rpc_read_float(slave_id, read_addr);
}

// =================================================================================
// 3. MAIN SETUP & LOOP
// =================================================================================

void setup() {
    // 1. Init Modbus Port
    Monitor.begin(IsunaDriver::BAUDRATE);
    
    // 2. Init RPC Bridge
    Bridge.begin(); 

    // 3. Bind Functions
    Bridge.provide("isuna_read", rpc_read_float);
    Bridge.provide("isuna_write", rpc_write_float);
    Bridge.provide("isuna_rw", rpc_read_write);
}

void loop() {
    // Keep the bridge alive
    Bridge.update(); 
}
