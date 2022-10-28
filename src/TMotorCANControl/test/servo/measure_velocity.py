from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import csv
import time
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo import TMotorManager_servo
from TMotorCANControl.Serial_manager_servo import *
import serial


# v_max = 0 # ERPM
# v_step = 1000 # ERPM
# num_iters = int(v_max/v_step)+1
# v_test_array = [i*v_step for i in range(num_iters)]

v_test_array = [5000]
num_iters = len(v_test_array)

step_duration = 3.0 # seconds

print("Measuring velocities: {}".format(v_test_array))
with open("Measuring_velocities_{}_ERPM.csv".format(v_test_array[-1]),'w') as fd:
    writer = csv.writer(fd)
    writer.writerow(['timestamp (epoch)', 'loop_time (s)', 'velocity (ERPM)'])
    with TMotorManager_servo(motor_type='AK80-9', motor_ID=0, CSV_file="log.csv") as dev:
        with serial.Serial("/dev/ttyUSB4", 961200, timeout=100) as ser:
            params = servo_motor_serial_state()
            ser.write(bytearray(startup_sequence()))
            ser.write(bytearray(set_motor_parameter_return_format_all()))
            loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
            dev.enter_velocity_control()
            i = 0
            t_next = step_duration
            print("driving at: {}ERPM".format(v_test_array[i]))
            for t in loop:
                if t >= t_next:
                    t_next += step_duration
                    i += 1
                    print("driving at: {}ERPM".format(v_test_array[i]))

                dev.θd = v_test_array[i]
                dev.update()
                data = read_packet(ser)
                if len(data):
                    params = parse_motor_parameters(data)
                    
                ser.write(bytearray(get_motor_parameters()))
                writer.writerow([time.time(), t, dev.θd, params.input_current, params.input_voltage])
                # print("\r" + str(dev) + 'i_bus: ' + str(round(params.input_current)), end='')













