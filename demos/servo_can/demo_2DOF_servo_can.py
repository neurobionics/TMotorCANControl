from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_can import TMotorManager_servo_can
import time
import numpy as np

with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev1:
    with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=1, CSV_file="log.csv") as dev2:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        dev1.zero_position()
        dev2.zero_position()
        dev1.enter_position_control()
        dev2.enter_position_control()
        for t in loop:
            dev1.θd = np.sin(t) # rad/s
            dev2.θd = np.sin(t) # rad/s
            dev1.update()
            dev2.update()
            print("\r" + str(dev1),end='')
