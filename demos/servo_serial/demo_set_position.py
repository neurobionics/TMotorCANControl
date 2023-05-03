from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_serial import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np

pos = 0
vel = 6.28
acc = 1000.0

with TMotorManager_servo_serial(port='/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK60-6']) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)

        dev.set_zero_position()
        dev.update()
        dev.comm_set_multi_turn()
        dev.update()
        dev.enter_position_velocity_control()
        pos = dev.get_output_angle_radians()

        for t in loop:
            real_pos = dev.get_output_angle_radians()
            if np.abs(pos - real_pos) < 0.01:
                pos = float(input("\n Enter the angle in rad: "))
            dev.set_output_angle_radians(pos, vel, acc)
            dev.update()
            print(f"\r {dev}", end='')
