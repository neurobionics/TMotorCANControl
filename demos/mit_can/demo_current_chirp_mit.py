from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
from TMotorCANControl.TMotorManager_mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK80-9'
ID = 1


def chirp_demo(dev, amp=1.0, dt=0.001):
    print("Chirping ActPackA. Press CTRL-C to finish.")
    chirp = Chirp(250, 25, 1)
    dev.set_set_current_gains()
    
    print("Starting current chirp demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = dt, report=True, fade=0.1)
    for t in loop:
        dev.update()
        des_τ = loop.fade*amp*chirp.next(t)*3/3.7
        dev.torque = des_τ # a barely audible note

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        chirp_demo(dev, amp=1.0)
    print("done with chirp_demo()")