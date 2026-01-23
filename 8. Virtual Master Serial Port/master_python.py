"""
Project: Virtual Modem Simulator
Protocol: Modbus RTU (Binary)
Functionality: Sends Function Code 06 (Write Single Register) requests to a Windows Modbus Slave application
"""

import socket
import struct
import time

HOST_IP = "172.24.0.1"  
PORT    = 5020           

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos 
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def main():
    print(f"Dialing Windows App at {HOST_IP}:{PORT}...")
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((HOST_IP, PORT))
        print("CONNECTED! Sending data...")

        slave_id = 1
        func_code = 6 
        reg_addr = 0
        value = 500

        while True:
            frame = struct.pack('>B B H H', slave_id, func_code, reg_addr, value)
            
            full_packet = frame + calculate_crc(frame)
            
            print(f"[TX] Writing Value: {value} (Hex: {full_packet.hex().upper()})")
            
            s.send(full_packet)
            
            time.sleep(2)
            value += 1
            if value > 600: value = 500

    except ConnectionRefusedError:
        print("\n[ERROR] Connection Refused!")
        print("1. Is 'Modbus Slave' running?")
        print("2. Is the Port 5020?")
        print("3. Check Windows Firewall.")
    except TimeoutError:
        print("\n[ERROR] Connection Timed Out. Check the IP address.")
    except KeyboardInterrupt:
        print("\nStopping...")
        s.close()

if __name__ == "__main__":
    main()
