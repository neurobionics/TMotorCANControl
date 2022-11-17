from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.test.servo_serial.Serial_manager_servo import *
from TMotorCANControl.TMotorManager_servo_serial import *

duty = 0

Pdes = 0
Vdes = 0

P = 0.1
D = 0.0

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev:
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        dev.set_zero_position()
        dev.update()
        
        dev.enter_duty_cycle_control()
        
        for t in loop:
            Pdes = np.sin(2*t)
            duty = P*(Pdes - dev.get_output_angle_radians()) + D*(Vdes - dev.get_output_velocity_radians_per_second())
            dev.set_duty_cycle(duty)
            dev.update()
            print(f"\r {dev} | vq: {dev.vq}", end='')



        