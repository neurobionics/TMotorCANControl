from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.AdcManager import ADC_Manager
import time
import csv
import numpy as np
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo import TMotorManager_servo

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
v_max = 1000 # ERPM
v_step = 500 # ERPM
num_iters = int(v_max/v_step)+1
v_test_array = [i*v_step for i in range(num_iters)]
step_duration = 3.0 # seconds

iq_antagonist = 0

with open("Measuring_efficiency_{}_A_antagonist.csv".format(iq_antagonist),'w') as fd:
    writer = csv.writer(fd)
    writer.writerow(["timestamp (epoch)", "loop time (s)", "velocity (ERPM)", "ADC Voltage (V)", "Futek Torque (Nm)", "Antagonist Q-Current (A)"])
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
            adc.update()
            writer.writerow([time.time(), t, dev.θd, adc.volts, volt_to_torque(adc.volts, bias=bias), iq_antagonist])
            # print("\r" + str(dev), end='')













