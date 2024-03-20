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

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK60-6']) as dev:
        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_control()
        t_min1 = 0
        for t in loop:
            Pdes = 1*np.sin(t)
            cmd = P*(Pdes - dev.position ) + D*(Vdes - dev.velocity)
            dev.current_qaxis = -cmd
            dev.update()
            print(f"\r {dev}","T:",str(t-t_min1), end='')
            t_min1 = t
            

        