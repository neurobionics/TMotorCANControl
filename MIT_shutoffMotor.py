import TControl as tc
import can
motor_ID = 2

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except:
    print("Failed to open CAN bus.")
else:
    
    tc.send_setting_message(bus, motor_ID, tc.special_codes['exit_motor_control_mode'])