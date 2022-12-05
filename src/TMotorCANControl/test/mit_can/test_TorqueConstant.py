from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.mit_can import TMotorManager
from NeuroLocoMiddleware.AdcManager import ADC_Manager;
import numpy as np
import matplotlib.pyplot as plt
import csv
#from TMotorCANControl.src.TMotorCANControl.TMotorManager import LOG_VARIABLES

# CHANGE THESE TO MATCH YOUR DEVICE!


current_step_val = 10
trial_num = 1

Type = 'AK10-9'
ID = 2

# need desired winding current (below), measured winding current (log), measured torque (log), motor torque (log)

torque_rating = 100 # For Futek - 100 Nm = 5 V
max_voltage = 5 # 5 V corresponding to torque rating
filt_size = 1

test_log_variables = [
    "time",
    "q_axis_current",
    "ADC_Voltage",
    "measured_torque",
    "temperature"
]





def current_step(dev, adc, current_step_val, filt_size):
    dev.zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    dev.set_current_gains()
    
    torque_init = 0; # adc.zero_torque(torque_rating, max_voltage)
    voltage_raw = []

    print("Starting current step demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.004, report=False, fade=0)
    i = 0
    print("Starting Test")
    for t in loop:
        dev.update()
        adc.update()
        if t < 1.0 or t > 10.0: # 0-1 is off, 1-2 not counted, 2-9 trial, 9-10 not counted, >10 off
            dev.i = 0.0
        else:
            #this will range from -5.0 to 5.0 in 1.0 intervals 
            dev.i = current_step_val
        
        # voltage_raw.append(adc.volts)
        # if i >= filt_size:
        #     voltage = np.median(voltage_raw[i-filt_size:i])
        # else:
        voltage = adc.volts
        
        torque = adc.calc_torque(voltage, torque_rating, max_voltage,torque_init)
        current = dev.get_current_qaxis_amps()
        temperature = dev.get_temperature_celsius()
        log_vars = [t, current, voltage, torque, temperature]
        log_kt_test(csv_file_name, log_vars)

        if t > 15.0:
            break
        i += 1
    print("Done w/ Test. Closing Loop")
    del loop


def log_kt_test(csv_file_name, data):
     if csv_file_name is not None:
            with open(csv_file_name,'a') as fd:
                writer = csv.writer(fd)
                writer.writerow(data)

if __name__ == '__main__':

    adc = ADC_Manager()

    trial_num = 1
    

    with TMotorManager(motor_type=Type, motor_ID=ID) as dev:
        for i in range(5):
            time.sleep(1.5)
            print("Starting Trial "+str(trial_num))
            csv_file_name = "Logs/motor_constant_test_"+str(current_step_val)+"A_"+str(trial_num)+".csv"
            trial_num += 1
            if csv_file_name is not None:
                with open(csv_file_name,'w') as fd:
                    writer = csv.writer(fd)
                    writer.writerow(test_log_variables)
            current_step(dev, adc, current_step_val, filt_size)
            