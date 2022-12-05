from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_serial import *

current = 1

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.002, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_loop_control()
        
        for t in loop:
            dev.set_current_qaxis_amps(current)
            dev.update()
            print("\r" + str(dev), end='')

        