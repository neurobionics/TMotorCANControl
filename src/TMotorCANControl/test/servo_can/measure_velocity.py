from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.AdcManager import ADC_Manager
import csv
import time
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo
from TMotorCANControl.test.servo_serial.Serial_manager_servo import *
import serial


# v_max = 0 # ERPM
# v_step = 1000 # ERPM
# num_iters = int(v_max/v_step)+1
# v_test_array = [i*v_step for i in range(num_iters)]

# adc = ADC_Manager()
# torque_rating = 100  # 100 Nm = 5 V
# def volt_to_torque(volt, bias=0):
#     return (volt-2.5-bias)/2.5*torque_rating
# voltage_cal = []
# print("Calibrating Loadcell!!!")
# for i in range(500):
#     adc.update()
#     voltage_cal.append(adc.volts)
# avg_volt = np.mean(np.array(voltage_cal))
# bias = avg_volt - 2.5
# print("Bias: {} V".format(bias))

# duty_test_array = [0.1]
# num_iters = len(duty_test_array)

v_test_array = [0.25, 0.5, 0.75, 1] # , 12, 14, 16, 18, 20
num_iters = len(v_test_array)

v_arr = []

step_duration = 2 # seconds

print("Measuring velocities: {}".format(v_test_array))
with open("Measuring_velocities_{}_rps.csv".format(v_test_array[-1]),'w') as fd:
    writer = csv.writer(fd)
    writer.writerow(['epoch', 'loop time', 'angular velocity (rad/s)', 'i_bus', 'v_bus', 'iq', 'vq', 'CAN error code', 'Serial error code'])
    with TMotorManager_servo(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev:
        with serial.Serial("/dev/ttyUSB0", 961200, timeout=100) as ser:
            params = servo_motor_serial_state()
            ser.write(bytearray(startup_sequence()))
            ser.write(bytearray(set_motor_parameter_return_format_all()))
            ser.write(bytearray(set_multi_turn()))

            loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
            dev.enter_velocity_control()
            i = 0
            t_next = step_duration
            print("driving at: {} rad/s".format(v_test_array[i]))
            ser.flushInput()
            for t in loop:
                if t >= t_next:
                    print("θd_avg: {} rad/s".format(np.mean(np.array(v_arr[-int(0.5*t):]))))
                    i += 1
                    if (i >= num_iters):
                        break
                    t_next += step_duration
                    v_arr = []
                    print("driving at: {} rad/s".format(v_test_array[i]))
                
                # radians per second is a misnomer
                dev.θd = (v_test_array[i])
                dev.update()
                data = read_packet(ser)
                if len(data):
                    p = parse_motor_parameters(data)
                    if p.initialized:
                        params = p
                ser.write(bytearray(get_motor_parameters()))
                v_arr.append(dev.θd)
                # adc.update()
                writer.writerow([time.time(), t, dev.θd, params.input_current, params.input_voltage, params.iq_current, params.Vq, dev.get_motor_error_code(), params.error]) # adc.volts, volt_to_torque(adc.volts, bias=bias),
                # print("\r" + str(dev) + 'i_bus: ' + str(round(params.input_current)), end='')
            













