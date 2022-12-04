from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_serial import *

def chirp_demo(dev, amp=1.0, dt=0.005):
    print("Starting current chirp demo. Press ctrl+C to quit.")
    chirp = Chirp(50, 25, True)
    dev.enter_current_control()

    loop = SoftRealtimeLoop(dt = dt, report=True)
    for t in loop:
        dev.update()
        dev.iq = amp*chirp.next(t) # a barely audible note

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
    chirp_demo(dev, amp=1.0)
print("done with chirp_demo()")
