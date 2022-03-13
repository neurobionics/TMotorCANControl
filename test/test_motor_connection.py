from sys import path
path.append("/home/pi/TControl")
from TControl import TMotorManager

ID = 1
Type = 'AK80-9'

with TMotorManager(motor_type=Type, motor_ID=ID, CSV_file="log.csv") as motor3:
    print(motor3)