from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
import numpy as np
import time

# try to import the default version, if that fails import from local directory
try:
     from TMotorCANControl.TMotorManager import TMotorManager
except ModuleNotFoundError:
    from sys import path
    path.append("/home/pi/TMotorCANControl/src")
    from TMotorCANControl.TMotorManager import TMotorManager


with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as dev:
    dev.zero_position() # has a delay!
    time.sleep(1.75)
    dev.set_impedance_gains_real_unit_full_state_feedback(K=10,B=1)
    chirp_slow = Chirp(250, 200, 1)
    loop = SoftRealtimeLoop(dt = 0.001, report=True, fade=0)
    amp = 1.0
    
    # start in current mode
    state = 0
    dev.set_current_gains() 
    
    delta = 5
    t_next=delta
    print("(1 of 8) Setting torque to 0.0 Nm")

    for t in loop:
        dev.update()
        if(t > t_next):
            state += 1
            if state == 1:
                print("(2 of 8) Chirping")
                dev.set_current_gains()
            elif state == 2:
                print("(3 of 8) Zeroing Position K = 1.0 B =0.05")
                dev.set_impedance_gains_real_unit(K=1.0,B=0.05)
            elif state == 3:
                print("(4 of 8) Zeroing Position K = 10 B =0.5")
                dev.set_impedance_gains_real_unit(K=10.0,B=0.5)
            elif state == 4:
                print("(5 of 8) Tracking sinusoidal trajectory")
                dev.set_impedance_gains_real_unit(K=10.0,B=0.5)
            elif state == 5:
                print("(6 of 8) Full state feedback. Chirping during impedance control!")
                dev.set_impedance_gains_real_unit_full_state_feedback(K=10.0,B=0.5)
            elif state == 6:
                print("(7 of 8) Setting torque to 0.5 Nm")
                dev.set_current_gains()
            elif state == 7:
                print("(8 of 8) Setting current to -0.5 A")
                dev.set_current_gains()
            elif state == 8:
                print("Done! Press ctrl+C to exit.")
                dev.set_current_gains()
                dev.i = 0.0

            t_next += delta
                
        if state == 0:
            dev.τ = 0.0
        elif state == 1:
            dev.τ = loop.fade*amp*chirp_slow.next(t)*3/3.7
        elif state == 2:
            dev.θ = 0.0
        elif state == 3:
            dev.θ = 0.0
        elif state == 4:
            dev.θ = 0.5*np.sin(np.pi*t)
        elif state == 5:
            dev.θ = 0.5*np.sin(np.pi*t)
            dev.τ = loop.fade*amp*chirp_slow.next(t)*3/3.7
        elif state == 6:
            dev.τ = 0.5
        elif state == 7:
            dev.i = -0.5
        else:
            dev.i=0.0

    del loop