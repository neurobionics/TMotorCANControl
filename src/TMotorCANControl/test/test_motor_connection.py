from sys import path
from TMotorCANControl.TMotorManager import TMotorManager
import time

# CHANGE THESE TO MATCH YOUR dev!
ID = 3
Type = 'AK80-9'

with TMotorManager(dev_type=Type, dev_ID=ID, CSV_file="log.csv") as dev:
    if dev.check_can_connection():
        print("\ndev is successfully connected!\n")
    else:
        print("\ndev not connected. Check dev power, network wiring, and CAN bus connection.\n")
    