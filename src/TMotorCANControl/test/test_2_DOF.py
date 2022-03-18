from SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TControl")
from TControl import TMotorManager
import numpy as np
import time
from SysID import Chirp


# to use additional motors, simply add another with block
# remember to give each motor a different log name!
with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log_motor3.csv") as motor3:
    with TMotorManager(motor_type='AK80-9', motor_ID=2, CSV_file="log_motor2.csv") as motor2:
        motor3.zero_position() # has a delay!
        # no need to zero motor2's position because we never use it.
        time.sleep(1.5)
        motor3.set_impedance_gains_real_unit(K=10.0,B=0.5)
        motor2.set_current_gains()
        chirp = Chirp(250, 200, 0.5)
        amp = 10.0
        loop = SoftRealtimeLoop(dt = 0.005, report=True, fade=0)
        for t in loop:
            motor3.update()
            motor2.update()
            if t < 1.0:
                motor3.θ = 0.0
                motor2.τ = 0.0
            else:
                motor3.θ = (np.pi/2)*int(t)
                motor2.τ = loop.fade*amp*chirp.next(t)*3/3.7

        del loop