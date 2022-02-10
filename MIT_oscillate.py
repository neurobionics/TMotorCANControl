import TControl as tc
import can
import time
motor_ID = 2

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except:
    print("Failed to open CAN bus.")
else:
    v_des = 0.0
    i_des = 0.0
    Kp = 10.0
    Kd = 1.0

    tc.send_setting_message(bus, motor_ID, tc.special_codes['enter_motor_control_mode'])
    tc.send_setting_message(bus, motor_ID, tc.special_codes['zero_current_motor_position'])

    while True:
        tc.MIT_controller(bus, motor_ID, 0.0, v_des, Kp, Kd, i_des)
        time.sleep(2)
        tc.MIT_controller(bus, motor_ID, 3.14, v_des, Kp, Kd, i_des)
        time.sleep(2)