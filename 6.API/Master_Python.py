from arduino.app_utils import App, Bridge
import time

next_state = True

def perform_led_task():
    global next_state
    if next_state:
        command_str = "\x00"
    else:
        command_str = "\x01"

    reply_str = device.write_read(command_str).strip()
    print(f"{reply_str}")

    next_state = not next_state

class UartDevice:
    def write(self, data):
        Bridge.notify("uart_write", data)

    def read(self):
        return Bridge.call("uart_read", "")

    def write_read(self, command):
        return Bridge.call("uart_writeread", command)

device = UartDevice()

def user_loop():
    print("----------------new data circle----------------")

    device.write("Hi from Master")
    
    time.sleep(0.1) 
    
    messagefromread = device.read()
    print(f"{messagefromread}")

    perform_led_task()
    
    time.sleep(3)

App.run(user_loop=user_loop)
