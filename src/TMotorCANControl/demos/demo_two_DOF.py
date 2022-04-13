from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
ID_1 = 3
ID_2 = 2

Type_1 = 'AK80-9'
Type_2 = 'AK80-9'


def two_DOF(dev1,dev2):
    dev1.zero_position()
    dev2.zero_position()
    time.sleep(1.5) # wait for the motors to zero (~1 second)
    dev1.set_impedance_gains_real_unit(K=10.0,B=0.5)
    dev2.set_impedance_gains_real_unit(K=10.0,B=0.5)
    
    print("Starting 2 DOF demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.005, report=True, fade=0)
    for t in loop:
        dev1.update()
        dev2.update()
        if t < 1.0:
            dev1.θ = 0.0
            dev2.θ = 0.0
        else:
            dev1.θ = (np.pi/2)*int(t)
            dev2.θ = (np.pi/2)*int(t)

    del loop

if __name__ == '__main__':
    # to use additional motors, simply add another with block
    # remember to give each motor a different log name!
    with TMotorManager(motor_type=Type_1, motor_ID=ID_1, CSV_file="log_dev1.csv") as dev1:
        with TMotorManager(motor_type=Type_2, motor_ID=ID_2, CSV_file="log_dev2.csv") as dev2:
            two_DOF(dev1,dev2)