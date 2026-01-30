import time
import ctypes
import os
import sys

# =================================================================================
# 1. CONSTANTS
# =================================================================================
class IsunaConfig:
    SLAVE_ID_DCAC = 0x01
    REG_GRID_VOLTAGE    = 0x1000  # [Read] Float
    REG_MAX_CHARGE      = 0x3040  # [Write] Float

# =================================================================================
# 2. SHARED LIBRARY ADAPTER (Replaces IsunaRpcAdapter)
# =================================================================================
class IsunaNativeAdapter:
    """
    Wraps the C++ Shared Object (.so) file.
    """
    def __init__(self, port_name="/dev/ttyUSB0"):
        # 1. Locate the compiled C++ library
        lib_path = os.path.join(os.getcwd(), "isuna_core.so")
        if not os.path.exists(lib_path):
            print(f"[FATAL] Library not found at {lib_path}")
            print("Did you compile it? g++ -shared -o isuna_core.so -fPIC isuna_core.cpp")
            sys.exit(1)
            
        self.lib = ctypes.CDLL(lib_path)

        # 2. Define Argument and Return types for safety.
        self.lib.isuna_read_float.argtypes = [ctypes.c_int, ctypes.c_int]
        self.lib.isuna_read_float.restype = ctypes.c_float

        self.lib.isuna_write_float.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float]
        self.lib.isuna_write_float.restype = ctypes.c_int

        self.lib.isuna_rw.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_int]
        self.lib.isuna_rw.restype = ctypes.c_float

        self.lib.isuna_init.argtypes = [ctypes.c_char_p]
        self.lib.isuna_init.restype = ctypes.c_int

        # 3. Initialize the Serial Port
        print(f"[Init] Opening Port {port_name}...")
        c_port = port_name.encode('utf-8')
        success = self.lib.isuna_init(c_port)
        if not success:
            print("[FATAL] Failed to open Serial Port via C++")
            sys.exit(1)

    def read_float(self, slave_id: int, address: int) -> float:
        return float(self.lib.isuna_read_float(slave_id, address))

    def write_float(self, slave_id: int, address: int, value: float) -> bool:
        res = self.lib.isuna_write_float(slave_id, address, float(value))
        return (res == 1)

    def read_and_write(self, slave_id: int, w_addr: int, w_val: float, r_addr: int) -> float:
        return float(self.lib.isuna_rw(slave_id, w_addr, float(w_val), r_addr))

# =================================================================================
# 3. DEVICE SERVICE (Unchanged Logic!)
# =================================================================================
class IsunaDevice:
    def __init__(self, adapter):
        self.adapter = adapter

    def get_grid_voltage(self) -> float:
        val = self.adapter.read_float(IsunaConfig.SLAVE_ID_DCAC, IsunaConfig.REG_GRID_VOLTAGE)
        # Assuming error codes are negative large numbers
        if val < -9000: 
            raise IOError(f"Sensor Read Error Code: {val}")
        return val

    def set_max_charge(self, amps: float) -> bool:
        print(f"[CMD] Setting Max Charge to {amps} A...")
        return self.adapter.write_float(IsunaConfig.SLAVE_ID_DCAC, IsunaConfig.REG_MAX_CHARGE, amps)

    def atomic_update_charge_limit(self, amps: float) -> float:
        print(f"[CMD] Atomic Update: Set {amps}A -> Check Voltage")
        val = self.adapter.read_and_write(
            IsunaConfig.SLAVE_ID_DCAC, IsunaConfig.REG_MAX_CHARGE, amps, IsunaConfig.REG_GRID_VOLTAGE
        )
        if val < -9000:
            raise IOError(f"Atomic Transaction Failed: {val}")
        return val

# =================================================================================
# 4. MAIN ENTRY POINT
# =================================================================================

if __name__ == "__main__":
    print("--- ISUNA MODBUS CONTROLLER (RASPBERRY PI EDITION) ---")
    
    # SETUP: Identify your USB Dongle path!
    # Common names: /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyAMA0 (if using GPIO pins)
    SERIAL_PORT = "/dev/ttyUSB0" 

    try:
        adapter = IsunaNativeAdapter(SERIAL_PORT)
        device = IsunaDevice(adapter)

        # MAIN LOOP
        while True:
            print("\n--- Control Cycle Start ---")
            try:
                # 1. READ
                voltage = device.get_grid_voltage()
                print(f"[STATUS] Grid Voltage: {voltage:.2f} V")
                
                # 2. LOGIC
                if voltage > 240.0:
                    print("[WARN] Overvoltage! Lowering Charge Current...")
                    if device.set_max_charge(5.0):
                        print("[OK] Charge Limited to 5A")
                
                # 3. ATOMIC TEST
                if voltage < 220.0:
                    new_volts = device.atomic_update_charge_limit(10.0)
                    print(f"[ATOMIC] Updated. Verified Voltage: {new_volts:.2f} V")

            except IOError as e:
                print(f"[ALARM] {e}")
            except KeyboardInterrupt:
                print("\n[STOP] User stopped the process.")
                break
            
            time.sleep(2.0)

    except Exception as e:
        print(f"[FATAL ERROR] {e}")
