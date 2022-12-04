from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_serial import *
import numpy as np
from NeuroLocoMiddleware.StatProfiler import SSProfile
current = 0

Pdes = 0
Vdes = 0

P = 2
D = 0.3

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_loop_control()
        
        for t in loop:
            Pdes = 3*np.sin(t)
            cmd = P*(Pdes - dev.θ ) + D*(Vdes - dev.θd)
            dev.iq = cmd
            SSProfile('update').tic()
            dev.update()
            SSProfile('update').toc()
            # print(f"\n {Pdes - dev.θ} {cmd} {dev.iq} ", end='')

        