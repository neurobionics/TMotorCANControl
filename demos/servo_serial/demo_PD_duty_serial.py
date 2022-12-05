from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_serial import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np

duty = 0

Pdes = 0
Vdes = 0

P = 0.1
D = 0.001

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_duty_cycle_control()
        
        for t in loop:
            Pdes = np.sin(2*t)
            duty = P*(Pdes - dev.get_output_angle_radians()) + D*(Vdes - dev.get_output_velocity_radians_per_second())
            dev.set_duty_cycle_percent(duty)
            dev.update()
            print(f"\r {dev}", end='')



        