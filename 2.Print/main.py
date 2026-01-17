# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
# SPDX-License-Identifier: MPL-2.0

from arduino.app_utils import *
import time

def loop():
    # Do absolutely nothing but wait
    time.sleep(1)

# Start the app so the Arduino code can run
App.run(user_loop=loop)
