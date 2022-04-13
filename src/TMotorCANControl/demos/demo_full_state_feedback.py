from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
import numpy as np
import time
from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK80-9'
ID = 3

def full_state_feedback(dev):
    dev.zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit_full_state_feedback(K=10,B=1)
    chirp = Chirp(250, 200, 0.5)

    print("Starting full state feedback demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.001, report=True, fade=0)
    amp = 1.0
  
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

if __name__ == '__main__':
    with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as dev:
        full_state_feedback(dev)