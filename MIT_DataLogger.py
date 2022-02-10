import TControl as tc
import can
import time
import json
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
    data_log = []
    end_time = time.time() + 10.0

    while time.time() < end_time:
        tc.send_setting_message(bus, motor_ID, tc.special_codes['enter_motor_control_mode'])
        msg = bus.recv(timeout=0.5)
        if msg is not None:
            data_log.append( [ msg.data, tc.parse_MIT_message(msg) ] )


    json_string = json.dump(data_log)
    f = open('json_data.json', 'w')
    json.dump(json_string, f)
    

            