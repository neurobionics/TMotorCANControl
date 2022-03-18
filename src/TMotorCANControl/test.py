from TMotorManager import TMotorManager
import numpy as np
from SoftRealtimeLoop import SoftRealtimeLoop



with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as motor3:
        motor3.power_on()
        motor3.zero_position() # has a delay!
        motor3.set_impedance_gains_real_unit(K=10,B=0.5)
        
        loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
        for t in loop:
            motor3.update()
            if t < 1.0:
                motor3.i = 0.0
            elif t < 4:
                motor3.i = 0.25

        del loop