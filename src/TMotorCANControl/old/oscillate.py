import TControl as tc
import can
import time
motor_ID = 3

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except:
    print("Failed to open CAN bus.")
else:
    p_des = 0.0
    v_des = 0.0
    i_des = 0.0
    Kp = 7.0
    Kd = 0.8

    tc.power_on(bus, motor_ID)
    tc.zero(bus, motor_ID)
    time.sleep(0.1)

    end_time = time.time() + 10.0

    # while time.time() < end_time:
    #     msg = bus.recv(timeout=1)
    #     if msg is not None:
    #         print(tc.parse_MIT_message(msg))
    #     tc.power_on(bus, motor_ID)
    #     time.sleep(0.01)
    delta = 0.5
    while time.time() < end_time:
        msg = bus.recv(timeout=1)
        if msg is not None:
            print(tc.parse_MIT_message(msg))
        tc.MIT_controller(bus, motor_ID, 0.0, 1, 0.0, 0.8, 0.0)
        time.sleep(3)
        
        msg = bus.recv(timeout=1)
        if msg is not None:
            print(tc.parse_MIT_message(msg))
        tc.MIT_controller(bus, motor_ID, 0.0, 0.0, 0.0, 0.0, 0.0)
        time.sleep(3)



    tc.power_off(bus, motor_ID)