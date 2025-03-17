from TMotorCANControl.servo_serial import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

with TMotorManager_servo_serial(port='/dev/ttyUSB0', motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        dev.enter_idle_mode()
        for t in loop:
            dev.update()
            Pdes = np.sin(t)
            print(f"\r {dev}", end='')




