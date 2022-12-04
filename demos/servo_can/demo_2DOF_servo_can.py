from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_can import TMotorManager_servo_can
import time
import numpy as np

with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=0) as dev1:
    with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=1) as dev2:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        dev1.zero_position()
        dev2.zero_position()

        dev1.enter_position_control()
        dev2.enter_idle_mode()

        for t in loop:
            dev1.θ = dev2.θ 
            dev1.update()
            dev2.update()
            print(f"\r {dev}", end='')
