from TMotorCANControl.CAN_Manager_mit import *
import time
canman = CAN_Manager()

ID = 3
Type = "AK80-9"
canman.power_on(ID)
canman.MIT_controller(ID,Type,0,0,0,0,1.)
time.sleep(5)

canman.power_off(ID)


