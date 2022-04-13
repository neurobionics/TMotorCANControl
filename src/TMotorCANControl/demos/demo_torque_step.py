from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK80-9'
ID = 3


def torque_step(dev):
    dev.zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    dev.set_current_gains()
    
    print("Starting torque step demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.τ = 0.0
        else:
            dev.τ = 0.0

    del loop


if __name__ == '__main__':
    with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as dev:
        torque_step(dev)