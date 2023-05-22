from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time
import numpy as np


with TMotorManager_servo_can(motor_type='AK60-6', motor_ID=1) as dev:
    
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    dev.set_zero_position()
    dev.enter_position_control()
    for t in loop:
        # dev.position = 1
        dev.set_output_angle_radians(0.1,0.1,0.5)
        dev.update()
        print("\r" + str(dev),end='')
