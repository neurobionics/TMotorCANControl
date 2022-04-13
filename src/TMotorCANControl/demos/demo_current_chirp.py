from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK80-9'
ID = 3


def chirp_demo(dev, amp=1.0, dt=0.001):
    print("Chirping ActPackA. Press CTRL-C to finish.")
    chirp = Chirp(250, 25, 1)
    dev.set_current_gains()
    
    print("Starting current chirp demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = dt, report=True, fade=0.1)
    for t in loop:
        dev.update()
        des_τ = loop.fade*amp*chirp.next(t)*3/3.7
        dev.τ = des_τ # a barely audible note
        # print(t, des_τ)

if __name__ == '__main__':
    with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as dev:
        chirp_demo(dev, amp=1.0)
    print("done with chirp_demo()")