from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
try:
     from TMotorCANControl.TMotorManager import TMotorManager
except ModuleNotFoundError:
    from sys import path
    path.append("/home/pi/TMotorCANControl/src")
    from TMotorCANControl.TMotorManager import TMotorManager
import time

logvars = [
    "output_angle", 
    "output_velocity", 
    "output_acceleration",
    "current",
    "output_torque",
    "motor_angle", 
    "motor_velocity", 
    "motor_acceleration", 
    "motor_torque"
]

with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv", log_vars=logvars) as dev:
    dev.zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit(K=10,B=0.5)

    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.ϕ = 0.0
        else:
            dev.ϕ = 10.0

    del loop