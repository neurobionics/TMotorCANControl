from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK80-9'
ID = 3

def position_tracking(dev):
    dev.zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit(K=10,B=0.5)

    print("Starting position tracking demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.θ = 0.0
        else:
            dev.θ = 0.5*np.sin(np.pi*t)
    
    del loop

if __name__ == '__main__':
    with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as dev:
        position_tracking(dev)