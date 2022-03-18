from sys import path
path.append("/home/pi/TControl")
from TControl import TMotorManager
from SoftRealtimeLoop import SoftRealtimeLoop
import time

# CHANGE THESE TO MATCH YOUR MOTOR!
ID = 3
Type = 'AK80-9'

with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as motor:
    if motor.check_can_connection():
        print("\nMotor is successfully connected!\n")
    else:
        print("\nMotor not connected. Check motor power, network wiring, and CAN bus connection.\n")
    