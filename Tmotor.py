import can
import numpy as np
import time


debug = True # set to False for final version
motor_on = False
motor_ID = 1

control_modes = { 'duty' : 0, 'current_loop' : 1, 'current_break' : 2, 'velocity' : 3, 'position' : 4, 'set_origin' : 5, 'position_velocity_accel' : 6}
special_codes = {
    'enter_motor_control_mode' : [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC ],
    'exit_motor_control_mode' : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD],
    'zero_current_motor_position' : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
}


def send_servo_control_message(bus, motor_id, control_mode, data):
# Set up arbitration ID to contain the control mode in the proper place
# Check that length of data isn’t more than 8 bytes (no buffer overflow attacks allowed!)
# Try to send the CAN message
# Return success or error
    DLC = len(data)
    assert (DLC <= 8), ('Data too long in message: ' + control_mode + " for motor: " + str(motor_id))
    arbitration_id = motor_id | (control_modes[control_mode] << 8 )
    if debug:
        print('ID: ' + str(hex(arbitration_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
    
    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended=True)
    try:
        bus.send(message)
        if debug:
            print("    Message sent on {}".format(bus.channel_info))
    except can.CanError:
        if debug:
            print("    Message NOT sent")


def send_setting_message(bus, motor_id, data):
    DLC = len(data)
    assert (DLC <= 8), ('Data too long in message for motor: ' + str(motor_id))
    
    if debug:
        print('ID: ' + str(hex(motor_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
    
    message = can.Message(arbitration_id=motor_id, data=data, is_extended=False)
    try:
        bus.send(message)
        if debug:
            print("    Message sent on {}".format(bus.channel_info))
    except can.CanError:
        if debug:
            print("    Message NOT sent")
    

def parse_servo_message(message):
    # Takes in data from a CAN message
    # Returns a tuple (position, speed, current, temp, error)
    
    # arbitration_id = message.arbitration_id
    data = bytes(message.data)

    position = float( np.int16(data[0] << 8 | data[1]) ) * 0.1
    speed = float( np.int16(data[2] << 8 | data[3]) ) * 10.0
    current = float( np.int16(data[4] << 8 | data[5]) ) * 0.01
    temp = float(np.int8(data[6]))
    error = np.int8(data[7])
    motor = str(message.arbitration_id)
    if debug:
        print('motor: ' + str(motor))
        print('Position: ' + str(position))
        print('Speed: ' + str(speed))
        print('Current: ' + str(current))
        print('Temp: ' + str(temp))
        print('Error: ' + str(error))

    return (motor, position, speed, current, temp, error)

    
def power_on(bus, motor_id):
    send_setting_message(bus, motor_id, special_codes['enter_motor_control_mode'])
        

def power_off(bus, motor_id):
    send_setting_message(bus, motor_id, special_codes['enter_motor_control_mode'])
    

def set_duty_cycle(bus, motor_id, duty):
    if duty > 1.0:
        duty = 1.0
    elif duty < -1.0:
        duty = -1.0

    duty_bytes = int(duty*100000.0).to_bytes()
    send_servo_control_message(bus, motor_id, control_modes['duty'], duty_bytes)


def set_position_deg(bus, motor_id, position):
    if position > 36000.0:
        position = 36000.0
    elif position < -36000.0:
        position = -36000.0

    position_bytes = int(position * 1000000.0).to_bytes()
    send_servo_control_message(bus, motor_id, control_modes['position'], position_bytes)


if __name__ == "__main__":

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except:
        print("Failed to initialize bus.")

    send_setting_message(bus, motor_ID, special_codes['enter_motor_control_mode'])
    time.sleep(1)
    send_servo_control_message(bus, motor_ID,'position', [0,0,0,0]) # go to position 0
 
    while True:
        msg = bus.recv(timeout=1.0)
        (position, speed, current, temp, error) = parse_servo_message(msg)




    





            


            
        









































