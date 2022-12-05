from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_serial import *

pos = 0

with TMotorManager_servo_serial(port='/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        
        dev.set_zero_position()
        dev.update()

        dev.enter_position_control()
        # time.sleep(1)
        for t in loop:
            pos = np.sin(t)
            pos = 10
            dev.set_output_angle_radians(pos, vel=1000, acc=100)
            dev.update()
            print("\r" + str(dev), end='')

        