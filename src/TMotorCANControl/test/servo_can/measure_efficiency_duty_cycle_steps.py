from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.AdcManager import ADC_Manager
import time
import csv
import numpy as np
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo
from TMotorCANControl.test.servo_serial.Serial_manager_servo import *
import serial


torque_rating = 100  # 100 Nm = 5 V
def volt_to_torque(volt, bias=0):
    return (volt-2.5-bias)/2.5*torque_rating

bias = 0
with ADC_Manager('ADC_backup_log.csv') as adc:
    adc.update()
    voltage_cal = []
    print("Calibrating Loadcell!!!")
    for i in range(500):
        adc.update()
        voltage_cal.append(adc.volts)
    avg_volt = np.mean(np.array(voltage_cal))
    bias = avg_volt - 2.5
    print("Bias: {} V".format(bias))
    adc.update()


# v_min always 0
duty_test_array = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95]
# pre_duty_test_array = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95]
# duty_test_array = []
# for d in pre_duty_test_array:
#     duty_test_array.append(d)
#     duty_test_array.append(0.0)

num_iters = len(duty_test_array)
step_duration = 1.0 # seconds

ERPM_to_RadPs = 2*np.pi/180/60 # (2/21)*9*(1/60)*(np.pi/180)

iq_antagonist = 0
 
with open("Measuring_efficiency_{}_A_antagonist{}.csv".format(iq_antagonist,time.time()),'w') as fd:
    writer = csv.writer(fd)
    writer.writerow(["timestamp (epoch)", "loop time (s)", "velocity (ERPM)", "velocity (Rad/S)", "ADC Voltage (V)", "Futek Torque (Nm)", "Antagonist Q-Current (A)", "i_bus", "v_bus", "v_q", "i_q", "duty", "mosfet_temp","i_d","v_d"])
    
    with TMotorManager_servo(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev:
        with serial.Serial("/dev/ttyUSB0", 961200, timeout=100) as ser:
            with ADC_Manager('ADC_backup_log.csv') as adc:
                adc.update()
                params = servo_motor_serial_state()
                ser.write(bytearray(startup_sequence()))
                ser.write(bytearray(set_motor_parameter_return_format_all()))

                loop = SoftRealtimeLoop(dt=0.1, report=True, fade=0.0)
                dev.enter_duty_cycle_control()
                i = 0
                t_next = step_duration
                print("testing with: {} V".format(duty_test_array[i]))
                time.sleep(0.1)
                for t in loop:
                    adc.update()

                    if t >= t_next:
                        t_next += step_duration
                        i += 1
                        if i < num_iters:
                            print("testing with: {} V".format(duty_test_array[i]))
                        else:
                            break

                    dev.set_duty_cycle(duty_test_array[i])
                    dev.update()
                    
                    # put this into an "update" function later and run ascynch
                    data = read_packet(ser)
                    if len(data):
                        p = parse_motor_parameters(data)
                        if p.initialized:
                            params = p
                    ser.write(bytearray(get_motor_parameters()))

                    writer.writerow([time.time(), t, dev.θd, dev.θd*ERPM_to_RadPs, adc.volts, volt_to_torque(adc.volts, bias=bias), iq_antagonist, params.input_current, params.input_voltage, params.Vq, params.iq_current, params.duty, params.mos_temperature, params.id_current, params.Vd])
                    # print("\r" + str(dev), end='')













