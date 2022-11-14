from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.Serial_manager_servo import *
from TMotorCANControl.TMotorManager_servo_serial import *

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        time.sleep(0.1)
        for t in loop:
            dev.enter_idle_mode()
            dev.update()
            print("\r" + str(dev), end='')


