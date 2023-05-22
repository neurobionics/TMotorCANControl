from TMotorCANControl.servo_serial import TMotorManager_servo_serial, Servo_Params_Serial

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK60-6'
ID = 1

with TMotorManager_servo_serial(motor_params=Servo_Params_Serial['AK60-6'], motor_ID=ID) as dev:
    if dev.check_connection():
        print("\nmotor is successfully connected!\n")
    else:
        print("\nmotor not connected. Check dev power, network wiring, and Serial bus connection.\n")
    