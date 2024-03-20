from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_serial import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

baud=921600
port='/dev/ttyUSB0'
motor_params = Servo_Params_Serial['AK60-6']

with TMotorManager_servo_serial(port=port, baud=baud, motor_params=motor_params) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        dev.enter_idle_mode()
        for t in loop:
            dev.update()
            Pdes = np.sin(t)
            print(f"\r {dev}", end='')




