from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# try to import the default version, if that fails import from local directory
try:
     from TMotorCANControl.TMotorManager import TMotorManager
except ModuleNotFoundError:
    from sys import path
    path.append("/home/pi/TMotorCANControl/src")
    from TMotorCANControl.TMotorManager import TMotorManager
import numpy as np
import time
from NeuroLocoMiddleware.SysID import Chirp

with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as dev:
    dev.zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit_full_state_feedback(K=10,B=1)
    chirp = Chirp(250, 200, 0.5)
    loop = SoftRealtimeLoop(dt = 0.001, report=True, fade=0)
    amp = 3.0
  
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.τ = 0.0
            dev.θ = 0.0
        else:
            des_τ = loop.fade*amp*chirp.next(t)*3/3.7
            dev.τ = des_τ
            dev.θ = (np.pi/2)*int(t)

    del loop