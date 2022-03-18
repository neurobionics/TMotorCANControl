from SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TControl")
from TControl import TMotorManager
import numpy as np
import time

with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as motor3:
    print(motor3)
    motor3.zero_position() # has a delay!
    time.sleep(1.5)
    motor3.set_impedance_gains_real_unit(K=10,B=0.5)
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        motor3.update()
        if t < 1.0:
            motor3.θ = 0.0
        elif t < 4:
            motor3.θ = 0.5*np.sin(np.pi*t)
        else:
            motor3.θ = 2*np.sin(0.5*np.pi*t)

    del loop