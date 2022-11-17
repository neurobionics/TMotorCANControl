import can
import os
from collections import namedtuple
from math import isfinite
import numpy as np
# Control mode contain {0,1,2,3,4,5,6,7} Seven eigenvalues correspond to seven control modes
# respectively
# Duty cycle mode: 0
# Current loop mode: 1
# Current brake mode: 2
# Velocity mode: 3
# Position mode: 4
# Set origin mode:5
# Position velocity loop mode :6
# Parameter dictionary for each specific motor that can be controlled with this library
# Thresholds are in the datasheet for the motor on cubemars.com
# Verified Error codes for servo motor

Servo_Params = {
        'ERROR_CODES':{
            0 : 'No Error',
            1 : 'Over temperature fault',
            2 : 'Over current fault',
            3 : 'Over voltage fault',
            4 : 'Under voltage fault',
            5 : 'Encoder fault',
            6 : 'Phase current unbalance fault (The hardware may be damaged)'
        },
        'AK10-9':{
            'P_min' : -32000,#-3200 deg
            'P_max' : 32000,#3200 deg
            'V_min' : -100000,#-100000 rpm electrical speed
            'V_max' : 100000,# 100000 rpm electrical speed
            'Curr_min':-1500,#-60A is the acutal limit but set to -15A
            'Curr_max':1500,#60A is the acutal limit but set to 15A
            'T_min' : -15,#NM
            'T_max' : 15,#NM
            'Kt_TMotor' : 0.16, # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 0.59, # UNTESTED CONSTANT!
            'Kt_actual': 0.206, # UNTESTED CONSTANT!
            'GEAR_RATIO': 9.0, 
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'AK80-9':{
            'P_min' : -32000,#-3200 deg
            'P_max' : 32000,#3200 deg
            'V_min' : -32000,#-320000 rpm electrical speed
            'V_max' : 32000,# 320000 rpm electrical speed
            'Curr_min':-1500,#-60A is the acutal limit but set to -15A
            'Curr_max':1500,#60A is the acutal limit but set to 15A
            'T_min' : -30,#NM
            'T_max' : 30,#NM
            'Kt_TMotor' : 0.091, # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 0.59,
            'Kt_actual': 0.115,
            'GEAR_RATIO': 9.0, 
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'CAN_PACKET_ID':{

            'CAN_PACKET_SET_DUTY':0, #Motor runs in duty cycle mode
            'CAN_PACKET_SET_CURRENT':1, #Motor runs in current loop mode
            'CAN_PACKET_SET_CURRENT_BRAKE':2, #Motor current brake mode operation
            'CAN_PACKET_SET_RPM':3, #Motor runs in current loop mode
            'CAN_PACKET_SET_POS':4, #Motor runs in position loop mode
            'CAN_PACKET_SET_ORIGIN_HERE':5, #Set origin mode
            'CAN_PACKET_SET_POS_SPD':6, #Position velocity loop mode
        },
}
"""
A dictionary with the parameters needed to control the motor
"""

crc16_tab = [0x0000, 0x1021, 0x2042,0x3063, 0x4084,
0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c,0xd1ad,
0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294,0x72f7,
0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff,0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,0xa56a,
0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653,0x2672,
0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a,0x9719,
0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886,0x78a7,
0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af,0x8948,
0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71,0x0a50,
0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58,0xbb3b,
0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60,0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,0x7e97,
0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f,0xefbe,
0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9,0xb1ca,
0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2,0x20e3,
0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da,0xc33d,
0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235,0x5214,
0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f,0xd52c,
0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424,0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,0x26d3,
0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c,0xc96d,
0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865,0x7806,
0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f,0xfb1e,
0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16,0x0af1,
0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa,0xad8b,
0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83,0x1ce0,
0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9,0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0]

class servo_motor_state:
    """Data structure to store and update motor states"""
    def __init__(self,position, velocity, current, temperature, error, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
        """
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(self, position, velocity, current, temperature, error, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state):
        """
        Sets this motor state object's values to those of another motor state object.

        Args:
            other_motor_state: The other motor state object with values to set this motor state object's values to.
        """
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration

    def __str__(self):
        return 'Position: {} | Velocity: {} | Current: {} | Temperature: {} | Error: {}'.format(self.position, self.velocity, self.current, self.temperature, self.error)

# Data structure to store MIT_command that will be sent upon update
class servo_command:
    """Data structure to store MIT_command that will be sent upon update"""
    def __init__(self, position, velocity, current, duty):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            kp: Position gain
            kd: Velocity gain
            current: Current in amps
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.duty = duty

# # motor state from the controller, uneditable named tuple
# servo_motor_state = namedtuple('motor_state', 'position velocity current temperature error')
# """
# Motor state from the controller, uneditable named tuple
# """

# python-can listener object, with handler to be called upon reception of a message on the CAN bus
class motorListener(can.Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""
    def __init__(self, canman, motor):
        """
        Sets stores can manager and motor object references
        
        Args:
            canman: The CanManager object to get messages from
            motor: The TMotorCANManager object to update
        """
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg):
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        args:
            msg: A python-can CAN message
        """
        data = bytes(msg.data)
        ID = msg.arbitration_id & 0x00000FF
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_servo_message(data))

# A class to manage the low level CAN communication protocols
class CAN_Manager_servo(object):
    """A class to manage the low level CAN communication protocols"""
    debug = False
    """
    Set to true to display every message sent and recieved for debugging.
    """
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    """
    Used to keep track of one instantation of the class to make a singleton object
    """
    
    def __new__(cls):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager_servo, cls).__new__(cls)
            print("Initializing CAN Manager")
            # verify the CAN bus is currently down
            os.system( 'sudo /sbin/ip link set can0 down' )
            # start the CAN bus back up
            os.system( 'sudo /sbin/ip link set can0 up type can bitrate 500000' )
            # # increase transmit buffer length
            # os.system( 'sudo ifconfig can0 txqueuelen 1000')
            # create a python-can bus object
            cls._instance.bus = can.interface.Bus(channel='can0', bustype='socketcan')# bustype='socketcan_native')
            # create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self):
        """
        ALl initialization happens in __new__
        """
        pass
        
    def __del__(self):
        """
        # shut down the CAN bus when the object is deleted
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
        """
        os.system( 'sudo /sbin/ip link set can0 down' ) 

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor(self, motor):
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception

        Args:
            motor: The TMotorManager object to be subscribed to the notifier
        """
        self.notifier.add_listener(motorListener(self, motor))

#* Buffer information for servo mode data manipulation

#******************START****************************#
    # Buffer allocation for 16 bit
    @staticmethod
    def buffer_append_int16( buffer,number, index):
        """
        buffer size for int 16

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: Size of the buffer.
        """
        buffer.append((number >> 8)&(0x00FF))
        buffer.append((number)&(0x00FF))
    
    # Buffer allocation for unsigned 16 bit
    @staticmethod
    def buffer_append_uint16( buffer,number, index):
        """
        buffer size for Uint 16

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: Size of the buffer.
        """
        buffer.append((number >> 8)&(0x00FF))
        buffer.append((number)&(0x00FF))
       
    # Buffer allocation for 32 bit
    @staticmethod
    def buffer_append_int32( buffer,number, index):
        """
        buffer size for int 32

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: Size of the buffer.
        """
        buffer.append((number >> 24)&(0x000000FF))
        buffer.append((number >> 16)&(0x000000FF))
        buffer.append((number >> 8)&(0x000000FF))
        buffer.append((number)&(0x000000FF))

    # Buffer allocation for 32 bit
    @staticmethod
    def buffer_append_uint32( buffer,number, index):
        """
        buffer size for uint 32

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: Size of the buffer.
        """
        buffer.append((number >> 24)&(0x000000FF))
        buffer.append((number >> 16)&(0x000000FF))
        buffer.append((number >> 8)&(0x000000FF))
        buffer.append((number)&(0x000000FF))

    # Buffer allocation for 64 bit
    @staticmethod
    def buffer_append_int64( buffer,number, index):
        """
        buffer size for int 64

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: Size of the buffer.
        """
        buffer.append((number >> 56)&(0x00000000000000FF))
        buffer.append((number >> 48)&(0x00000000000000FF))
        buffer.append((number >> 40)&(0x00000000000000FF))
        buffer.append((number >> 31)&(0x00000000000000FF))
        buffer.append((number >> 24)&(0x00000000000000FF))
        buffer.append((number >> 16)&(0x00000000000000FF))
        buffer.append((number >> 8)&(0x00000000000000FF))
        buffer.append((number)&(0x00000000000000FF))

    # Buffer allocation for Unsigned 64 bit
    @staticmethod
    def buffer_append_uint64( buffer,number, index):
        """
        buffer size for uint 64

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: Size of the buffer.
        """
        buffer.append((number >> 56)&(0x00000000000000FF))
        buffer.append((number >> 48)&(0x00000000000000FF))
        buffer.append((number >> 40)&(0x00000000000000FF))
        buffer.append((number >> 31)&(0x00000000000000FF))
        buffer.append((number >> 24)&(0x00000000000000FF))
        buffer.append((number >> 16)&(0x00000000000000FF))
        buffer.append((number >> 8)&(0x00000000000000FF))
        buffer.append((number)&(0x00000000000000FF))


#******************END****************************#

    #CRC-Used for serial communication only!!
    @staticmethod
    def crc16( buf,len) :
        cksum = 0
        for i in range(len):
            cksum = crc16_tab[(((cksum >> 8) ^ buf+1) & 0xFF)] ^ (cksum << 8)
        return cksum
    
#* Sends data via CAN
    # sends a message to the motor (when the motor is in Servo mode)
    def send_servo_message(self, motor_id, data,data_len):
        """
        Sends a Servo Mode message to the motor, with a header of motor_id and data array of data

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
        """
        DLC = data_len
        assert (DLC <= 8), ('Data too long in message for motor ' + str(motor_id))
        
        if self.debug:
            print('ID: ' + str(hex(motor_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
        
        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info) )
        except can.CanError as e:
            if self.debug:
                print("    Message NOT sent: " + e.message)

    # send the power on code
    def power_on(self, motor_id):
        """
        Sends the power on code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
            Data: This is obtained from the datasheet.
        """

        self.send_servo_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC], 0)
        
    # send the power off code
    def power_off(self, motor_id):
        """
        Sends the power off code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_servo_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD], 0)


#* Code for the working of different modes in servo mode. 
   
    #* ********************START*******************************************#
    #TODO: Controller id vs motorID
    # Send Servo control message for duty cycle mode
    #*Duty cycle mode: duty cycle voltage is specified for a given motor, similar to squarewave drive mode
    def comm_can_set_duty(self, controller_id, duty):
        send_index = 0
        buffer=[]
        self.buffer_append_int32(buffer, int(duty * 100000.0), send_index)
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_DUTY'] << 8), buffer, send_index)

    # Send Servo control message for current loop mode
    #*Current loop mode: given the Iq current specified by the motor, the motor output torque = Iq *KT, so it can be used as a torque loop
    def comm_can_set_current(self,controller_id, current):
        send_index = 0
        buffer=[]
        self.buffer_append_int32(buffer, int(current * 1000.0), send_index)
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_CURRENT'] << 8), buffer, send_index)

    # Send Servo control message for current brake mode
    #*Current brake mode: the motor is fixed at the current position by the specified brake current given by the motor (pay attention to the motor temperature when using)
    def comm_can_set_cb(self, controller_id, current):
        send_index = 0
        buffer=[]
        self.buffer_append_int32(buffer, int(current * 1000.0), send_index)
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_CURRENT_BRAKE'] << 8), buffer, send_index)
        
    # Send Servo control message for Velocity mode
    #*Velocity mode: the speed specified by the given motor
    def comm_can_set_rpm(self,controller_id, rpm):
        send_index = 0
        buffer=[]
        self.buffer_append_int32(buffer, int(rpm), send_index)
        self.send_servo_message(controller_id| (Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_RPM'] << 8), buffer, send_index)
    
    # Send Servo control message for Position Loop mode
    #*Position mode: Given the specified position of the motor, the motor will run to the specified position, (default speed 12000erpm acceleration 40000erpm)
    def comm_can_set_pos(self, controller_id, pos):
        send_index = 0
        buffer=[]
        self.buffer_append_int32(buffer, int(pos * 1000000.0), send_index)
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_POS'] << 8), buffer, send_index)
    
    #Set origin mode
    #*0 means setting the temporary origin (power failure elimination), 1 means setting the permanent zero point (automatic parameter saving), 2means restoring the default zero point (automatic parameter saving)
    def comm_can_set_origin(self, controller_id, set_origin_mode) :
        send_index=0
        buffer=[]
        self.send_servo_message(controller_id |(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_ORIGIN_HERE'] << 8), buffer, send_index)

    #Position and Velocity Loop Mode
    #* Check documentation
    def comm_can_set_pos_spd(self, controller_id, pos, spd, RPA ):
        send_index = 0
        send_index1 = 0
        buffer=[]
        self.buffer_append_int32(buffer, (pos * 10000.0), send_index)
        self.buffer_append_int16(buffer,spd, send_index1)
        self.buffer_append_int16(buffer,RPA, send_index1)
        self.send_servo_message(controller_id |(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_POS_SPD'] << 8), buffer, send_index)

   # TODO: Servo control mode for Setting origin and postion+velocity modes.
    #* **************************END************************************************#
 


#*****************Parsing message data********************************#
    def parse_servo_message(self, data):
        # using numpy to convert signed/unsigned integers
        pos_int = np.int16(data[0] << 8 | data[1])
        spd_int = np.int16(data[2] << 8 | data[3])
        cur_int = np.int16(data[4] << 8 | data[5])
        motor_pos= ( pos_int * 0.1)
        motor_spd= (float)( spd_int * 10.0) #motor speed
        motor_cur= (float)( cur_int * 0.01) #motor current
        motor_temp= np.int16(data[6])  #motor temperature
        motor_error= data[7] #motor error mode
        if self.debug:
            print(data)
            print('  Position: ' + str(motor_pos))
            print('  Velocity: ' + str(motor_spd))
            print('  Current: ' + str(motor_cur))
            print('  Temp: ' + str(motor_temp))
            print('  Error: ' + str(motor_error))
            
        return servo_motor_state(motor_pos, motor_spd,motor_cur,motor_temp, motor_error, 0)



if __name__ == '__main__':
    buff1 = []
    CAN_Manager_servo.buffer_append_int32(buff1,-900,0)
    print('{}'.format(', '.join(hex(d) for d in buff1)) )

    # b'\x83\x00\x00\x00\xff\xff\x1d\x00' looks like 600A but really small amount of negative cause 2's compliment
    test_data = [0x83,0x00,0x00,0x00,0xff,0xff,0x1d,0x00]
    canman = CAN_Manager_servo()
    testState = canman.parse_servo_message(test_data)
    print(testState)