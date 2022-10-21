from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.CAN_manager_servo import CAN_Manager_servo
import time

canman = CAN_Manager_servo()
loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
canman.power_on(1)
for t in loop:
    
    if t > 2:
        
        canman.comm_can_set_rpm(1,0)
        msg = canman.bus.recv(timeout=0.001)
        if msg is not None:
            print(msg) # canman.parse_servo_message(msg)
        else:
            print("timeout")

    if t > 5:
        canman.power_off(1)
        break







