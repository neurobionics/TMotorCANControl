import TControl as tc
import can
motor_ID = 2

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except:
    print("Failed to open CAN bus.")
else:
    tc.power_off(bus, motor_ID)