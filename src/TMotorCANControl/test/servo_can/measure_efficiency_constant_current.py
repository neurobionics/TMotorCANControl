from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.AdcManager import ADC_Manager
import time
import csv
import numpy as np
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo

adc = ADC_Manager()
torque_rating = 100  # 100 Nm = 5 V
def volt_to_torque(volt, bias=0):
    return (volt-2.5-bias)/2.5*torque_rating

voltage_cal = []
print("Calibrating Loadcell!!!")
for i in range(500):
    adc.update()
    voltage_cal.append(adc.volts)
avg_volt = np.mean(np.array(voltage_cal))
bias = avg_volt - 2.5
print("Bias: {} V".format(bias))

# v_min always 0
i_max = 2 # A
i_step = 1 # A
num_iters = int(i_max/i_step)+1
i_test_array = [i*i_step for i in range(num_iters)]
step_duration = 2.0 # seconds

v_antagonist = 0

with open("Measuring_efficiency_{}_rad_per_s_antagonist.csv".format(v_antagonist),'w') as fd:
    writer = csv.writer(fd)
    writer.writerow(["timestamp (epoch)", "loop time (s)", "velocity (ERPM)", "Current (A)","ADC Voltage (V)", "Futek Torque (Nm)", "Antagonist Velocity (rad/s)"])
    with TMotorManager_servo(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev:
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        dev.enter_current_control()
        i = 0
        t_next = step_duration
        for t in loop:
            if t >= t_next:
                t_next += step_duration
                i += 1
                print("Testing {} A".format(i_test_array[i]))
            dev.i = i_test_array[i]
            dev.update()
            adc.update()
            writer.writerow([time.time(), t, dev.θd, adc.volts, volt_to_torque(adc.volts, bias=bias), v_antagonist])
            # print("\r" + str(dev), end='')













