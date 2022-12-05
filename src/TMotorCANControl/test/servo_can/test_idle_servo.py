from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo
import time


with TMotorManager_servo(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev:
    # dev.zero_position() # has a delay!
    # time.sleep(1.5)
    # dev.set_current_gains()
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    dev.enter_current_control()
    for t in loop:
        dev.i = 0
        dev.update()
        print("\r" + str(dev),end='')