from sys import path
path.append("/home/pi/TMotorCANControl/src")
from TMotorCANControl.TMotorManager import TMotorManager
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.AdcManager import ADC_Manager
import csv


bias = 0.5896742308538513 # Nm
test_time = 10.0 # s
torque = 0.0 # Nm (starting torque)
max_torque = 25.0 # Nm

τ_per_sec = max_torque/test_time

csv_file_name = "current_comp_test_1.csv"
with ADC_Manager(csv_file_name="dummyLog") as adc:
    with TMotorManager(motor_ID=3,CSV_file=None,use_torque_compensation=False) as dev:
        with open(csv_file_name,'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(["loop_time","motor_torque_command","adc_measured_torque","motor_measured_torque","motor_measured_current","motor_measured_speed"])
            
            dev.set_current_gains()
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
            i = 0
            for t in loop:
                
                print("\rTesting torque: " + str(torque) + " Nm",end="")
                adc.update()
                dev.update()
                writer.writerow([t, torque, adc.get_torque() - bias, dev.τ, dev.i, dev.θd])
                if t <= test_time:
                    torque = τ_per_sec*t
                    dev.τ = torque
                else:
                    dev.τ = 0.0
                
            del loop