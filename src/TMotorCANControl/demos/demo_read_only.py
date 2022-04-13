from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK80-9'
ID = 3

def read_only(dev):
    dev.zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    
    print("Starting read only demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    
    for t in loop:
        dev.update()
        print("\r" + str(dev), end='')


if __name__ == '__main__':
    with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as dev:
        read_only(dev)