from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_serial import *
import numpy as np
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
current = 0

Pdes = 0
Vdes = 0

P = 2
D = 0.3

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_control()
        
        for t in loop:
            Pdes = 3*np.sin(t)
            cmd = P*(Pdes - dev.θ ) + D*(Vdes - dev.θd)
            dev.current_qaxis = cmd
            dev.update()
            print(f"\r {dev}", end='')
            

        