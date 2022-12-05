from operator import mod
from NeuroLocoMiddleware.AdcManager import ADC_Manager;
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
import numpy as np
import matplotlib.pyplot as plt
import csv

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

loop = SoftRealtimeLoop(dt = 0.005, report=True, fade=0)
torque_init = volt_to_torque(adc.volts,bias)

time_array = []
voltage_array = []
torque_array = []
current_array = []
velocity_array = []
temperature_array = []

iq_des = 0 # Amps

with TMotorManager_servo('AK80-9',0) as dev:
    dev.enter_current_control()
    for t in loop:
        if t > 10:
            break
        else:
            dev.i = iq_des
        time_array.append(t)
        
        adc.update()
        dev.update()
        voltage_array.append(adc.volts)
        torque_array.append(volt_to_torque(adc.volts, bias=bias))
        current_array.append(dev.i)
        velocity_array.append(dev.θd)
        temperature_array.append(dev.T)
        print("\r" + str(dev),end='')
            
del loop
plt.plot(time_array, torque_array)
plt.savefig("torque_test_script_{}_A.png".format(iq_des))

with open("torque_test_script_{}_A.csv".format(iq_des),'w') as fd:
    writer = csv.writer(fd)
    for i in range(len(voltage_array)):
        writer.writerow([time_array[i], voltage_array[i], torque_array[i], current_array[i], velocity_array[i], temperature_array[i]])


