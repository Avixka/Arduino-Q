import time
from arduino.app_utils import App, Bridge

led_state = False

def print_function(message):
    print(f"Received from MCU: {message}")
    return True 
  
Bridge.provide("print_msg", print_function)

def loop():
    global led_state
    time.sleep(1)
  
    led_state = not led_state
    
    Bridge.call("blink_led", led_state)

App.run(user_loop=loop)
