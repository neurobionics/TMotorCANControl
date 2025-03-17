from TMotorCANControl.servo_serial import *

with TMotorManager_servo_serial(port= '/dev/ttyUSB0', motor_params=Servo_Params_Serial["AK80-9"]) as dev:
    if dev.check_connection():
        print("\nmotor is successfully connected!\n")
    else:
        print("\nmotor not connected. Check dev power, network wiring, and Serial bus connection.\n")
    