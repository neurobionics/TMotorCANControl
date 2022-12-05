from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_can import TMotorManager_servo_can


def chirp_demo(dev, amp=1.0, dt=0.001):
    print("Starting current chirp demo. Press ctrl+C to quit.")
    chirp = Chirp(300, 200, True)
    dev.enter_current_control()

    loop = SoftRealtimeLoop(dt = dt, report=True)
    for t in loop:
        dev.update()
        dev.current_qaxis = amp*chirp.next(t) # a barely audible note

with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=0) as dev:
    chirp_demo(dev, amp=3.0)
print("done with chirp_demo()")
