from TMotorCANControl.servo_serial import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np

pos = 0

with TMotorManager_servo_serial(port='/dev/ttyUSB0', motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        
        dev.set_zero_position()
        dev.update()

        dev.enter_position_control()
        for t in loop:
            pos = 2*np.sin(t)
            dev.set_output_angle_radians(pos)
            dev.update()
            print(f"\r {dev}", end='')

        