from TMotorCANControl.TMotorManager import TMotorManager

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK10-9'
ID = 2

with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file=None) as dev:
    if dev.check_can_connection():
        print("\nmotor is successfully connected!\n")
    else:
        print("\nmotor not connected. Check dev power, network wiring, and CAN bus connection.\n")
    