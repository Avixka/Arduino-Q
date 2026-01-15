from arduino.app_utils import App, Bridge
import time

# API Definition
class UartDevice:
    def write(self, data):
        Bridge.notify("uart_write", data)

    def read(self):
        response = Bridge.call("uart_read", "")
        return response

    def write_read(self, command):
        reply = Bridge.call("uart_writeread", command)
        return reply

device = UartDevice()

def user_loop():
    print("--- Script Started ---")
    
    # Wait 2 seconds for connections to stabilize before starting
    time.sleep(2)
    
    while True:
        try:
            print("\n--- NEW CYCLE ---")

            # TEST 1: uart_write
            print("1. Testing WRITE...")
            device.write("LED_ON")
            time.sleep(0.5) 
            device.write("LED_OFF")

            # TEST 2: uart_writeread
            print("2. Testing WRITE_READ...")
            # We add a specific try-block here in case the timeout is strict
            response = device.write_read("PING")
            print(f"   -> Result: {response}") 

            # TEST 3: uart_read
            print("3. Testing READ...")
            background_data = device.read()
            print(f"   -> Result: {background_data}")
            
            # Successful cycle, wait 2 seconds
            time.sleep(2)

        except Exception as e:
            # THIS IS THE FIX:
            # If any error happens, code jumps here instead of closing.
            print(f"!!! CRITICAL ERROR: {e} !!!")
            print("Retrying in 2 seconds...")
            time.sleep(2)

App.run(user_loop=user_loop)
