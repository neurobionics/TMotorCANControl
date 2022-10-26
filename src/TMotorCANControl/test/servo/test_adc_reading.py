from operator import mod
from NeuroLocoMiddleware.AdcManager import ADC_Manager;
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
import numpy as np
import matplotlib.pyplot as plt

adc = ADC_Manager()

torque_rating = 100  # 100 Nm = 5 V
len = 5000
voltage_array = []
torque_array = []

voltage_raw = []
time_list = []

filt_size = 50



print("Calibrating Loadcell!!!")

for i in range(500):
    adc.update()
    


loop = SoftRealtimeLoop(dt = 0.002, report=False, fade=0)   

torque_init = (adc.volts-2.5)/2.5*torque_rating

i = 0
for t in loop:
    adc.update()
    voltage_raw.append(adc.volts)
    if i >= filt_size:
        # print(voltage_raw[i-filt_size:i])
        voltage = np.median(voltage_raw[i-filt_size:i])

        torque = (voltage-2.5)/2.5*torque_rating-torque_init

        voltage_array.append(voltage) 
        torque_array.append(torque)
        time_list.append(t)
        
        if i % 100 == 0:
            print(f"Loadcell- Voltage: {voltage} Torque: {torque}")
    
    i += 1
    
    if i > len:
        break
del loop
plt.plot(time_list, torque_array)
plt.savefig("torque_test_script.png")