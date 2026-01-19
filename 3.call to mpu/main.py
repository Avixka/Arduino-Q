import time

from arduino.app_utils import App, Bridge

def print_function(message):
  print(f"Recived from MCU:{message}")

Bridge.provide("key_word",print_function)

#App.run()
def other_works():
    time.sleep(1)

App.run(user_loop=other_works)
