from sys import path
from TMotorCANControl.TMotorManager import TMotorManager
from SoftRealtimeLoop import SoftRealtimeLoop
from AdcManager import ADC_Manager
import csv


bias = 0.5896742308538513 # Nm
delay = 5.0 # s
test_torques = [0.0,0.25,-0.25,1.0,-1.0] # Nm
torque = test_torques[0] # Nm


csv_file_name = "log_adc_and_motor.csv"
with ADC_Manager(csv_file_name="dummyLog") as adc:
    with TMotorManager(motor_ID=3,CSV_file=None) as dev:
        with open(csv_file_name,'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(["loop_time","motor_torque_command","adc_measured_torque","motor_measured_torque","motor_measured_current","motor_measured_speed"])
            
            dev.set_current_gains()
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
            t_next = delay
            i = 0
            for t in loop:
                if t > t_next:
                    i += 1
                    torque = test_torques[i]
                    t_next += delay
                    print("Testing torque: " + str(torque) + " Nm")
                adc.update()
                dev.update()
                writer.writerow([t, torque, adc.get_torque() - bias, dev.τ, dev.i, dev.θd])
                dev.τ = torque
                
                
            del loop