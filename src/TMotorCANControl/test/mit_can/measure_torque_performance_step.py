from sys import path
path.append("/home/pi/TMotorCANControl/src")
from TMotorCANControl.mit_can import TMotorManager
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.AdcManager import ADC_Manager
import csv


bias = 0.5896742308538513 # Nm
test_torque = -15.0

csv_file_name = "saved_logs/sys_ID_final/trial8.csv"
with ADC_Manager(csv_file_name="dummyLog") as adc:
    with TMotorManager(motor_ID=3,CSV_file=None) as dev:
        with open(csv_file_name,'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(["loop_time","motor_torque_command","adc_measured_torque","motor_measured_torque","motor_measured_current","motor_measured_speed"])
            
            dev.set_current_gains()
            loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
            
            i = 0
            print("Testing torque: " + str(test_torque) + " Nm")
            for t in loop:
                adc.update()
                dev.update()
                writer.writerow([t, test_torque, adc.get_torque() - bias, dev.τ, dev.i, dev.θd])
                dev.τ = test_torque
                
            del loop