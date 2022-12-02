from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.TMotorManager_mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK10-9'
ID = 1

def speed_step(dev):
    dev.zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    dev.set_speed_gains(kd=3.0)
    
    print("Starting speed step demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.θd = 0.0
        else:
            dev.θd = 1.0
            
    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as dev:
        speed_step(dev)