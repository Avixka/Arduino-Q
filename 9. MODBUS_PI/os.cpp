/*
 * PROJECT: ISUNA Modbus Bridge (Linux Core)
 * PLATFORM: Raspberry Pi (Linux)
 * COMPILER: g++
 */

#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <chrono>       // For timing
#include <thread>       // For delays

// Global File Descriptor for the Serial Port
int serial_fd = -1;

// =================================================================================
// 1. LINUX UTILITIES (Replacements for Arduino functions)
// =================================================================================

// Get milliseconds since start
uint32_t millis() {
    using namespace std::chrono;
    static auto start_time = steady_clock::now();
    auto now = steady_clock::now();
    return duration_cast<milliseconds>(now - start_time).count();
}

// Standard blocking delay
void delay_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// =================================================================================
// 2. MODBUS DRIVER LAYER (Your Original Logic)
// =================================================================================
namespace IsunaDriver {
    
    // CRC16 Calculation (Unchanged)
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

    void floatToBytes(float val, uint8_t* dest) {
        union { float f; uint32_t i; } u;
        u.f = val;
        // Big Endian conversion
        dest[0] = (u.i >> 24) & 0xFF;
        dest[1] = (u.i >> 16) & 0xFF;
        dest[2] = (u.i >> 8) & 0xFF;
        dest[3] = u.i & 0xFF;
    }

    float bytesToFloat(const uint8_t* src) {
        union { uint32_t i; float f; } u;
        u.i = ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
              ((uint32_t)src[2] << 8)  | ((uint32_t)src[3]);
        return u.f;
    }

    // Transaction Handler (Ported to Linux)
    int sendRawTransaction(uint8_t* frame, size_t len, uint8_t* response, size_t maxResp) {
        if (serial_fd < 0) {
            std::cerr << "[Error] Serial port not opened!" << std::endl;
            return 0;
        }

        // 1. Flush Buffer (TCIFLUSH)
        tcflush(serial_fd, TCIFLUSH);

        // 2. Write Frame
        int written = write(serial_fd, frame, len);
        if (written < 0) {
            std::cerr << "[Error] Write failed" << std::endl;
            return 0;
        }

        // 3. Read Response with Timeout
        // Note: Linux is fast. We use a simple polling loop to mimic Arduino's behavior.
        uint32_t start = millis();
        size_t idx = 0;
        uint8_t buf[1];

        while (millis() - start < 200) { // 200ms Timeout
            int n = read(serial_fd, buf, 1);
            if (n > 0) {
                if (idx < maxResp) {
                    response[idx++] = buf[0];
                }
            } else {
                // No data available yet, sleep tiny bit to save CPU
                std::this_thread::sleep_for(std::chrono::microseconds(100)); 
            }
        }
        
        return idx; 
    }
}

// =================================================================================
// 3. EXPORTED API (The "New" RPC)
// =================================================================================
extern "C" {

    // INIT: Call this from Python first! 
    // e.g., port_name = "/dev/ttyUSB0" or "/dev/ttyS0"
    int isuna_init(const char* port_name) {
        serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd == -1) {
            std::cerr << "[Init] Unable to open port: " << port_name << std::endl;
            return 0;
        }

        // Configure Serial Port (115200 8N1)
        struct termios options;
        tcgetattr(serial_fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver
        options.c_cflag &= ~PARENB;          // No parity
        options.c_cflag &= ~CSTOPB;          // 1 Stop bit
        options.c_cflag &= ~CSIZE;           // Mask character size
        options.c_cflag |= CS8;              // 8 Data bits
        
        // Raw Input Mode (Disable Canonical)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST; // Raw output

        // Set non-blocking read
        fcntl(serial_fd, F_SETFL, FNDELAY);

        tcsetattr(serial_fd, TCSANOW, &options);
        std::cout << "[Init] Serial Port Opened Successfully: " << port_name << std::endl;
        return 1;
    }

    float isuna_read_float(int slave_id, int address) {
        uint8_t tx[8];
        uint8_t rx[32]; 

        tx[0] = (uint8_t)slave_id;
        tx[1] = (uint8_t)0x03; 
        tx[2] = (address >> 8) & 0xFF;
        tx[3] = address & 0xFF;
        tx[4] = 0x00; tx[5] = 0x02; 

        uint16_t crc = IsunaDriver::calculateCRC(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;

        int received = IsunaDriver::sendRawTransaction(tx, 8, rx, 32);

        if (received < 9) return -9999.0f; 
        
        uint16_t rxCrc = IsunaDriver::calculateCRC(rx, received - 2);
        if (rx[received-2] != (rxCrc & 0xFF) || rx[received-1] != (rxCrc >> 8)) {
            return -9998.0f; // CRC Error
        }

        return IsunaDriver::bytesToFloat(&rx[3]);
    }

    int isuna_write_float(int slave_id, int address, float value) {
        uint8_t tx[13];
        uint8_t rx[8];

        tx[0] = (uint8_t)slave_id;
        tx[1] = (uint8_t)0x10;
        tx[2] = (address >> 8) & 0xFF;
        tx[3] = address & 0xFF;
        tx[4] = 0x00; tx[5] = 0x02; 
        tx[6] = 0x04; 
        
        IsunaDriver::floatToBytes(value, &tx[7]);

        uint16_t crc = IsunaDriver::calculateCRC(tx, 11);
        tx[11] = crc & 0xFF;
        tx[12] = (crc >> 8) & 0xFF;

        int received = IsunaDriver::sendRawTransaction(tx, 13, rx, 8);

        if (received < 8) return 0; 
        if (rx[1] == 0x10) return 1; 
        return 0;
    }

    float isuna_rw(int slave_id, int w_addr, float w_val, int r_addr) {
        int writeSuccess = isuna_write_float(slave_id, w_addr, w_val);
        if (!writeSuccess) return -9997.0f; 

        delay_ms(20); // Safety buffer
        return isuna_read_float(slave_id, r_addr);
    }
}
