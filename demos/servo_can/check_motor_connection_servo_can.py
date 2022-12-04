from TMotorCANControl.TMotorManager_servo_can import TMotorManager_servo_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK10-9'
ID = 2

with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    if dev.check_can_connection():
        print("\nmotor is successfully connected!\n")
    else:
        print("\nmotor not connected. Check dev power, network wiring, and CAN bus connection.\n")
    