from TMotorCANControl.servo_serial import TMotorManager_servo_serial

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK10-9'
ID = 0

with TMotorManager_servo_serial(motor_type=Type, motor_ID=ID) as dev:
    if dev.check_connection():
        print("\nmotor is successfully connected!\n")
    else:
        print("\nmotor not connected. Check dev power, network wiring, and Serial bus connection.\n")
    