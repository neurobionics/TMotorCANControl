from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_can import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.StatProfiler import SSProfile
import numpy as np

Pdes = 0
Vdes = 0

P = 0.1
D = 0.0

with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=0) as dev:
    loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0.0)
    dev.set_zero_position()
    
    dev.update()
    dev.enter_duty_cycle_control()
    time.sleep(1)
    
    for t in loop:
        Pdes = np.sin(t)
        dev.set_duty_cycle_percent(-P*(Pdes - dev.position) + D*(Vdes - dev.velocity))
        dev.update()
        print(f"\r {dev}", end='')

        