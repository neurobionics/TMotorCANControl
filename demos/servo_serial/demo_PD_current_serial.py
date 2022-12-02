from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_serial import *
import numpy as np
current = 0

Pdes = 0
Vdes = 0

P = 2
D = 0.01

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_loop_control()
        
        for t in loop:
            Pdes = np.sin(t)
            dev.iq = P*(Pdes - dev.get_output_angle_radians()) + D*(Vdes - dev.get_output_velocity_radians_per_second())
            dev.update()
            # print(f"\n {dev} {Pdes} {dev.θ} ", end='')

        