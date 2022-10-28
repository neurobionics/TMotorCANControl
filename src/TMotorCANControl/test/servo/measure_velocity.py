from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import csv
import time
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo import TMotorManager_servo

# v_min always 0
v_max = 1000 # ERPM
v_step = 100 # ERPM
step_duration = 3.0 # seconds
num_iters = int(v_max/v_step)+1
v_test_array = [i*v_step for i in range(num_iters)]

print("Measuring velocities: {}".format(v_test_array))
with open("Measuring_velocities_{}_ERPM.csv".format(v_max),'w') as fd:
    writer = csv.writer(fd)
    writer.writerow(['timestamp (epoch)', 'loop_time (s)', 'velocity (ERPM)'])
    with TMotorManager_servo(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev:
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        dev.enter_velocity_control()
        i = 0
        t_next = step_duration
        for t in loop:
            if t >= t_next:
                t_next += step_duration
                i += 1
            dev.θd = v_test_array[i]
            dev.update()
            writer.writerow([time.time(), t, dev.θd])
            print("\r" + str(dev), end='')













