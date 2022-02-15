import TControl as tc
import can
import time
import os
import subprocess
motor_ID = 3

# try:
#     bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
# except:
#     print("Failed to open CAN bus.")
# else:


subprocess.call('dir')
subprocess.call(['sudo', '/sbin/ip', 'link', 'set', 'can0', 'down'])

for rate in range(125000, 1100000, 5000):
    print("Baud Rate: " + str(rate))
    time.sleep(0.01)
    subprocess.call(['sudo', '/sbin/ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', str(rate)])
    
    p = subprocess.Popen(['candump', 'can0'])
    
    subprocess.call(['cansend', 'can0', '003#FF.FF.FF.FF.FF.FF.FF.FD'])
    
    time.sleep(0.01)
    subprocess.call(['sudo', '/sbin/ip', 'link', 'set', 'can0', 'down'])





# rate = 1000000
# print("Baud Rate: " + str(rate))
# time.sleep(0.01)
# subprocess.call(['sudo', '/sbin/ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', str(rate)])
# #time.sleep(0.01)
# p = subprocess.Popen(['candump', 'can0'])
# #subprocess.run(["candump", "can0"])

# #time.sleep(0.01)
# subprocess.call(['cansend', 'can0', '003#FF.FF.FF.FF.FF.FF.FF.FD'])
# #time.sleep(0.01)
# # stdout, stderr = p.communicate()
# # print(stdout)
# time.sleep(0.01)
# subprocess.call(['sudo', '/sbin/ip', 'link', 'set', 'can0', 'down'])
    
    
