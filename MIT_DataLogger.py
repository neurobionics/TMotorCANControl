import TControl as tc
import can
import time
motor_ID = 2


def handle_message(msg):
    tc.parse_MIT_message(msg)
    


try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except:
    print("Failed to open CAN bus.")
else:
    csvListener = can.CSVWriter('csv_data.csv', append=False)
    canPrinter = can.Printer()
    notifier = can.Notifier([csvListener,canPrinter,handle_message])
    
    tc.power_on(bus, motor_ID)
    tc.zero(bus, motor_ID)
    
    end_time = time.time() + 2.0

    while time.time() < end_time:
        tc.power_on(bus, motor_ID)
        time.sleep(0.05)

    notifier.stop()
    tc.zero(bus, motor_ID)
        
    

            