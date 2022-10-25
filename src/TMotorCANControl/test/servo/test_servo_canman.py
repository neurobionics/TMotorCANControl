from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.CAN_manager_servo import CAN_Manager_servo
import time

ID = 0

canman = CAN_Manager_servo()
# loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
canman.power_on(ID)
t = 0.0
dt= 0.01
while (t < 10.0):
    
    canman.comm_can_set_current(ID,0.05)
    msg = canman.bus.recv(timeout=0.005)
    if msg is not None:
        # print(msg)
        print(canman.parse_servo_message(msg.data))
    t += dt







