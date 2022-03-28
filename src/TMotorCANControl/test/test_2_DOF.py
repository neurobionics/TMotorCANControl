from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from TMotorCANControl.TMotorManager import TMotorManager
import numpy as np
import time
from NeuroLocoMiddleware.SysID import Chirp


# to use additional motors, simply add another with block
# remember to give each motor a different log name!
with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log_dev1.csv") as dev1:
    with TMotorManager(motor_type='AK80-9', motor_ID=2, CSV_file="log_dev2.csv") as dev2:
        dev1.zero_position() # has a delay!
        # no need to zero dev2's position because we never use it.
        time.sleep(1.5)
        dev1.set_impedance_gains_real_unit(K=10.0,B=0.5)
        dev2.set_current_gains()
        chirp = Chirp(250, 200, 0.5)
        amp = 10.0
        loop = SoftRealtimeLoop(dt = 0.005, report=True, fade=0)
        for t in loop:
            dev1.update()
            dev2.update()
            if t < 1.0:
                dev1.θ = 0.0
                dev2.τ = 0.0
            else:
                dev1.θ = (np.pi/2)*int(t)
                dev2.τ = loop.fade*amp*chirp.next(t)*3/3.7

        del loop