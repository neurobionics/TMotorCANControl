import can
import numpy as np
import time

# global settings

debug = True # set to False for final version
motor_ID = 2

control_modes = { 
    'duty' : 0,
    'current_loop' : 1,
    'current_break' : 2,
    'velocity' : 3, 
    'position' : 4,
    'set_origin' : 5, 
    'position_velocity_accel' : 6
    }
    
special_codes = {
    'enter_motor_control_mode' : [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC ],
    'exit_motor_control_mode' : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD],
    'zero_current_motor_position' : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
}

Servo_Params = {
    'Duty_min': -1.0,
    'Duty_max': 1.0,
    'I_min': -60.0,
    'I_max': 60.0,
    'V_min': -100000.0,
    'V_max': 100000.0,
    'P_min': -36000.0,
    'P_max': 36000.0,
    'A_min': -100000.0,
    'A_max': 100000.0
}

MIT_Params = {
    'P_min' : -95.5,
    'P_max' : 95.5,
    'V_min' : -30.0,
    'V_max' : 30.0,
    'I_min' : -18.0,
    'I_max' : 18.0,
    'Kp_min': 0.0,
    'Kp_max': 500.0,
    'Kd_min': 0.0,
    'Kd_max': 5.0,
}


# Utility Functions

def send_servo_control_message(bus, motor_id, control_mode, data):
    DLC = len(data)
    assert (DLC <= 8), ('Data too long in message: ' + control_mode + " for motor: " + str(motor_id))
    arbitration_id = motor_id | (control_modes[control_mode] << 8 )
    if debug:
        print('ID: ' + str(hex(arbitration_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
    
    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
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
    
    message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    try:
        bus.send(message)
        if debug:
            print("    Message sent on {}".format(bus.channel_info))
    except can.CanError:
        if debug:
            print("    Message NOT sent")
    

def parse_servo_message(message):
    data = bytes(message.data)

    if debug:
        print(data)
    if(len(data) > 1):
        position = float( np.int16(data[0] << 8 | data[1]) ) * 0.1
    if(len(data) > 3):
        speed = float( np.int16(data[2] << 8 | data[3]) ) * 10.0
    if(len(data) > 5):
        current = float( np.int16(data[4] << 8 | data[5]) ) * 0.01
    if(len(data) > 6):
        temp = float(np.int8(data[6]))
    if(len(data) > 7):
        error = np.int8(data[7])

    motor = message.arbitration_id

    if debug:
        print('  ID: ' + str(motor))
        print('  Position: ' + str(position))
        print('  Speed: ' + str(speed))
        print('  Current: ' + str(current))
        print('  Temp: ' + str(temp))
        print('  Error: ' + str(error))

    return (motor, position, speed, current, temp, error)


def limit_value(value, min, max):
    if value > max:
        return max
    elif value < min:
        return min
    else:
        return value


    
# Servo Mode Functions

def set_duty_cycle(bus, motor_id, duty):
    duty = limit_value(duty, -1.0, 1.0)
    duty_bytes = int(duty*100000.0).to_bytes(4,byteorder = 'big')
    send_servo_control_message(bus, motor_id, control_modes['duty'], duty_bytes)

def set_position_deg(bus, motor_id, position):
    position = limit_value(position, -36000.0, 36000.0)
    position_bytes = int(position * 1000000.0).to_bytes(4,byteorder = 'big')
    send_servo_control_message(bus, motor_id, control_modes['position'], position_bytes)


# MIT Mode Functions
def power_on(bus, motor_id):
    send_setting_message(bus, motor_id, special_codes['enter_motor_control_mode'])
        
def power_off(bus, motor_id):
    send_setting_message(bus, motor_id, special_codes['exit_motor_control_mode'])

def float_to_uint(x,x_min,x_max,num_bits):
    x = limit_value(x,x_min,x_max)
    span = x_max-x_min
    # (x - x_min)*(2^num_bits)/span
    return int((x- x_min)*( float((1<<num_bits)/span)) )

def uint_to_float(x,x_min,x_max,num_bits):
    span = x_max-x_min
    # (x*span/(2^num_bits -1)) + x_min
    return float(x*span/((1<<num_bits)-1) + x_min)

def MIT_controller(bus, motor_id, position, velocity, Kp, Kd, I):

    position_uint16 = float_to_uint(position, MIT_Params['P_min'], MIT_Params['P_max'],16)
    velocity_uint12 = float_to_uint(velocity, MIT_Params['V_min'], MIT_Params['V_max'],12)
    Kp_uint12 = float_to_uint(Kp, MIT_Params['Kp_min'], MIT_Params['Kp_max'],12)
    Kd_uint12 = float_to_uint(Kd, MIT_Params['Kd_min'], MIT_Params['Kd_max'],12)
    I_uint12 = float_to_uint(I, MIT_Params['I_min'], MIT_Params['I_max'],12)

    data = [position_uint16 >> 8,
        position_uint16 & 0x00FF,
        (velocity_uint12) >> 4,
        (velocity_uint12&0x00F<<4) | (Kp_uint12) >> 8,
        (Kp_uint12&0x0FF),
        (Kd_uint12) >> 4,
        (Kd_uint12&0x00F<<4) | (I_uint12) >> 8,
        (I_uint12&0x0FF)
    ]

    send_setting_message(bus, motor_id, data)


def parse_MIT_message(message):
    
    data = bytes(message.data)
    if(len(data) < 8):
        print("Missing Data!!!")
        return

    else:
        motor = data[0]
        position = data[1] <<8 | data[2]
        velocity = data[3] << 8 | data[4]&0xF0
        current = data[4]&0x0F<<8 | data[5]
        temp = data[6]
        error = data[7]

    position = uint_to_float(position, MIT_Params['P_min'], MIT_Params['P_max'], 16)
    velocity = uint_to_float(velocity, MIT_Params['V_min'], MIT_Params['V_max'], 12)
    current = uint_to_float(current, MIT_Params['I_min'], MIT_Params['I_max'], 12)

    if debug:
        print(data)
        print('  ID: ' + str(motor))
        print('  Position: ' + str(position))
        print('  Velocity: ' + str(velocity))
        print('  Current: ' + str(current))
        print('  Temp: ' + str(temp))
        print('  Error: ' + str(error))

    return (motor, position, velocity, current, temp, error)
    

if __name__ == "__main__":

    p_des = 0.0
    v_des = 0.0
    i_des = 0.0
    Kp = 1
    Kd = 0.1

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except:
        print("Failed to open CAN bus.")
    else:

        send_setting_message(bus, motor_ID, special_codes['enter_motor_control_mode'])
        time.sleep(0.5)
        
        MIT_controller(bus, motor_ID, p_des, v_des, Kp, Kd, i_des)

        timeout = 0.1
        msg = bus.recv(timeout=1.0)
        while True:
            MIT_controller(bus, motor_ID, p_des, v_des, Kp, Kd, i_des)
            msg_buffer = []
            start = time.time()
            while(msg != None and not (timeout < (time.time() - start))):
                msg = bus.recv(timeout=1.0)
                msg_buffer.append(msg)

            if len(msg_buffer > 0):
                for message in msg_buffer:
                    parse_MIT_message(message)




    





            


            
        









































