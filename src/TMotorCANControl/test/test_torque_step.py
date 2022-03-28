try:
     from TMotorCANControl.TMotorManager import TMotorManager
except ModuleNotFoundError:
    from sys import path
    path.append("/home/pi/TMotorCANControl/src")
    from TMotorCANControl.TMotorManager import TMotorManager
    
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

import time


with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as dev:
    dev.zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_current_gains()
    
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.τ = 0.0
        else:
            dev.τ = 1.0

    del loop