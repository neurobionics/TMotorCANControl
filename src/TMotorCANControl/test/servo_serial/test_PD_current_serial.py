from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.test.servo_serial.Serial_manager_servo import *
from TMotorCANControl.servo_serial import *

current = 0

Pdes = 0
Vdes = 0

P = 2
D = 0.1

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_current_loop_control()
        
        for t in loop:
            current = P*(Pdes - dev.get_output_angle_radians()) + D*(Vdes - dev.get_output_velocity_radians_per_second())
            dev.iq = (current)
            dev.update()
            # print("\r" + str(dev), end='')

        