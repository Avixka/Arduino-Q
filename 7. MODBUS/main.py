"""
PROJECT: ISUNA Modbus Controller (MPU Side)
TARGET: Arduino UNO Q (Linux Environment)
ARCHITECTURE: 
  - IsunaConfig: Static configuration constants
  - IsunaRpcAdapter: Wrapper for the low-level Bridge calls
  - IsunaDevice: High-level business logic (The "Device Object")
  - App Context: Manages the main loop.
"""

import time
from arduino.app_utils import App, Bridge

# =================================================================================
# 1. CONSTANTS & CONFIGURATION
# =================================================================================
class IsunaConfig:
    SLAVE_ID_DCAC = 0x01
    
    # Register Map (from PDF)
    REG_GRID_VOLTAGE    = 0x1000  # [Read] Float
    REG_MAX_CHARGE      = 0x3040  # [Write] Float
    REG_BATTERY_SOC     = 0x100E  # [Read] Example

# =================================================================================
# 2. TRANSPORT ADAPTER (The "RPC Object")
# =================================================================================
class IsunaRpcAdapter:
    """
    Wraps the Arduino Bridge calls into a clean Python API.
    This creates the 'Object' you use to talk to the MCU.
    """
    def __init__(self):
        # Bridge is a singleton, so no socket connection logic needed here
        pass

    def read_float(self, slave_id: int, address: int) -> float:
        """Calls C++ 'isuna_read'"""
        try:
            # Arguments: (Function Name, SlaveID, Address)
            val = Bridge.call("isuna_read", slave_id, address)
            return float(val)
        except Exception as e:
            print(f"[RPC Error] Read Failed: {e}")
            return -9999.0

    def write_float(self, slave_id: int, address: int, value: float) -> bool:
        """Calls C++ 'isuna_write'"""
        try:
            # Arguments: (Function Name, SlaveID, Address, Value)
            result = Bridge.call("isuna_write", slave_id, address, float(value))
            return int(result) == 1
        except Exception as e:
            print(f"[RPC Error] Write Failed: {e}")
            return False

    def read_and_write(self, slave_id: int, w_addr: int, w_val: float, r_addr: int) -> float:
        """Calls C++ 'isuna_rw' (Atomic Transaction)"""
        try:
            # Arguments: (Function Name, SlaveID, WriteAddr, WriteVal, ReadAddr)
            result = Bridge.call("isuna_rw", slave_id, w_addr, float(w_val), r_addr)
            return float(result)
        except Exception as e:
            print(f"[RPC Error] Atomic RW Failed: {e}")
            return -9997.0

# =================================================================================
# 3. DEVICE SERVICE (The "Business Logic Object")
# =================================================================================
class IsunaDevice:
    """
    High-level representation of the ISUNA Device.
    You create an instance of this to control your hardware.
    """
    def __init__(self, adapter: IsunaRpcAdapter):
        self.rpc = adapter

    def get_grid_voltage(self) -> float:
        """Reads the Grid Voltage (Reg 0x1000)"""
        val = self.rpc.read_float(IsunaConfig.SLAVE_ID_DCAC, IsunaConfig.REG_GRID_VOLTAGE)
        if val < 0: 
            raise IOError(f"Sensor Read Error Code: {val}")
        return val

    def set_max_charge(self, amps: float) -> bool:
        """Sets the Max Charge Current (Reg 0x3040)"""
        print(f"[CMD] Setting Max Charge to {amps} A...")
        return self.rpc.write_float(IsunaConfig.SLAVE_ID_DCAC, IsunaConfig.REG_MAX_CHARGE, amps)

    def atomic_update_charge_limit(self, amps: float) -> float:
        """
        Sets the charge limit AND verifies the voltage in one locked transaction.
        Returns the verified voltage.
        """
        print(f"[CMD] Atomic Update: Set {amps}A -> Check Voltage")
        val = self.rpc.read_and_write(
            IsunaConfig.SLAVE_ID_DCAC,
            IsunaConfig.REG_MAX_CHARGE,    # Write Address
            amps,                          # Write Value
            IsunaConfig.REG_GRID_VOLTAGE   # Read Address
        )
        if val < 0:
            raise IOError(f"Atomic Transaction Failed: {val}")
        return val

# =================================================================================
# 4. MAIN APPLICATION ENTRY POINT
# =================================================================================

# We define the objects globally or inside the loop, depending on persistence needs.
# Here we initialize them once to simulate a professional service startup.

adapter = IsunaRpcAdapter()
device = IsunaDevice(adapter)

def user_loop():
    """
    This function is called repeatedly by App.run()
    """
    print("\n--- Control Cycle Start ---")
    
    try:
        # 1. READ OPERATION
        try:
            voltage = device.get_grid_voltage()
            print(f"[STATUS] Grid Voltage: {voltage:.2f} V")
            
            # 2. LOGIC & WRITE OPERATION
            if voltage > 240.0:
                print("[WARN] Overvoltage! Lowering Charge Current...")
                # Standard Write
                success = device.set_max_charge(5.0)
                if success: print("[OK] Charge Limited to 5A")
            
            # 3. ATOMIC READ-AND-WRITE OPERATION
            # Example: Periodically force a safety check update
            # This sets charge to 10A and gets the immediate voltage response
            # without risk of bus collision.
            if voltage < 220.0: 
                new_volts = device.atomic_update_charge_limit(10.0)
                print(f"[ATOMIC] Updated. Verified Voltage: {new_volts:.2f} V")

        except IOError as e:
            print(f"[ALARM] {e}")

    except Exception as fatal:
        print(f"[FATAL] Unexpected Error: {fatal}")

    # The App.run loop handles its own timing, but we can sleep here to slow it down
    time.sleep(2.0)

if __name__ == "__main__":
    print("--- ISUNA MODBUS CONTROLLER STARTING ---")
    # This hands control over to the Arduino App Utils framework
    App.run(user_loop=user_loop)
