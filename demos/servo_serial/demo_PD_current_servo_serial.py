from TMotorCANControl.servo_serial import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np

current = 0

Pdes = 0
Vdes = 0

P = 2
D = 0.3

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_control()
        
        for t in loop:
            Pdes = 3*np.sin(t)
            cmd = P*(Pdes - dev.position ) + D*(Vdes - dev.velocity)
            dev.current_qaxis = cmd
            dev.update()
            print(f"\r {dev}", end='')
            

        