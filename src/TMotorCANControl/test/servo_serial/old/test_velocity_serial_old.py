from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.test.servo_serial.Serial_manager_servo import *

end_time = 100
speed = 1000

with serial.Serial("/dev/ttyUSB0", 961200, timeout=100) as ser:
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        ser.write(bytearray(startup_sequence()))
        
        # not sure how to get position out of -360 to 360 degrees
        ser.write(bytearray(set_motor_parameter_return_format_all()))
        
        for t in loop:
            if t > end_time:
                # print(parse_motor_parameters(data))
                break
            else:
                data = read_packet(ser)
                # if len(data):
                    # print(data)
                    # hex_print(data)
                    # print(parse_motor_parameters(data))
                    # print('\x1B[13A',end='')
                
                # hex_print(cmd)
                # print(bytearray(cmd))
                ser.write(bytearray(set_speed(speed)))
                ser.write(bytearray(get_motor_parameters()))
                

        ser.write(bytearray(set_speed(0)))
                


