import can
import os
from collections import namedtuple
from math import isfinite

# Parameter dictionary for each specific motor that can be controlled with this library
# Thresholds are in the datasheet for the motor on cubemars.com
MIT_Params = {
        'AK80-9':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -50.0,
            'V_max' : 50.0,
            'I_min' : -18.0,
            'I_max' : 18.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'NM_PER_AMP': 0.146,
            'GEAR_RATIO': 9.0, # hence the 9 in the name
            'Use_derived_torque_constants': True, # true if you have a better model
            #         bias            nonlinear torque const multipliers  coulomb friction   gear friction
            'a_hat' : [7.35415941e-02, 6.26896231e-01, 2.65240487e-04,     2.96032614e-01,  7.08736309e-02]# [-5.86860385e-02,6.50840079e-01,3.47461078e-04,8.58635580e-01,2.93809281e-01]
        },
        'AK10-9':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -50.0,
            'V_max' : 50.0,
            'I_min' : -65.0,
            'I_max' : 65.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'NM_PER_AMP': 0.16*9.0, # UNTESTED CONSTANT!
            'GEAR_RATIO': 9.0, 
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'AK60-6':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -50.0,
            'V_max' : 50.0,
            'I_min' : -15.0,
            'I_max' : 15.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'NM_PER_AMP': 0.068*6.0, # UNTESTED CONSTANT!
            'GEAR_RATIO': 6.0, 
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'AK70-10':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -50.0,
            'V_max' : 50.0,
            'I_min' : -25.0,
            'I_max' : 25.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'NM_PER_AMP': 0.095*10.0, # UNTESTED CONSTANT!
            'GEAR_RATIO': 10.0,
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'AK80-6':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -76.0,
            'V_max' : 76.0,
            'I_min' : -12.0,
            'I_max' : 12.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'NM_PER_AMP': 0.091*6.0,  # UNTESTED CONSTANT!
            'GEAR_RATIO': 6.0, 
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'AK80-64':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -8.0,
            'V_max' : 8.0,
            'I_min' : -144.0,
            'I_max' : 144.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'NM_PER_AMP': 0.119*80.0, # UNTESTED CONSTANT!
            'GEAR_RATIO': 80.0,
            'Use_derived_torque_constants': False, # true if you have a better model
        }

}



class motor_state:
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

# Data structure to store MIT_command that will be sent upon update
class MIT_command:
    """Data structure to store MIT_command that will be sent upon update"""
    def __init__(self, position, velocity, kp, kd, current):
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
        self.kp = kp
        self.kd = kd
        self.current = current

# motor state from the controller, uneditable named tuple
MIT_motor_state = namedtuple('motor_state', 'position velocity current temperature error')

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
        ID = data[0]
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_MIT_message(data, self.motor.type))
            

# A class to manage the low level CAN communication protocols
class CAN_Manager(object):
    """A class to manage the low level CAN communication protocols"""
    debug = False
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    def __new__(cls):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            print("Initializing CAN Manager")
            # verify the CAN bus is currently down
            os.system( 'sudo /sbin/ip link set can0 down' )
            # start the CAN bus back up
            os.system( 'sudo /sbin/ip link set can0 up type can bitrate 1000000' )
            # create a python-can bus object
            cls._instance.bus = can.interface.Bus(channel='can0', bustype='socketcan')# bustype='socketcan_native')
            # create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self):
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


    # Locks value between min and max
    @staticmethod
    def limit_value(value, min, max):
        """
        Limits value to be between min and max

        Args:
            value: The value to be limited.
            min: The lowest number allowed (inclusive) for value
            max: The highest number allowed (inclusive) for value
        """
        if value > max:
            return max
        elif value < min:
            return min
        else:
            return value

    # interpolates a floating point number to fill some amount of the max size of unsigned int, 
    # as specified with the num_bits
    @staticmethod
    def float_to_uint(x,x_min,x_max,num_bits):
        """
        Interpolates a floating point number to an unsigned integer of num_bits length.
        A number of x_max will be the largest integer of num_bits, and x_min would be 0.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        x = CAN_Manager.limit_value(x,x_min,x_max)
        span = x_max-x_min
        # (x - x_min)*(2^num_bits)/span
        return int((x- x_min)*( float((1<<num_bits)/span)) )

    # undoes the above method
    @staticmethod
    def uint_to_float(x,x_min,x_max,num_bits):
        """
        Interpolates an unsigned integer of num_bits length to a floating point number between x_min and x_max.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max-x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x*span/((1<<num_bits)-1) + x_min)

    # sends a message to the motor (when the motor is in MIT mode)
    def send_MIT_message(self, motor_id, data):
        """
        Sends an MIT Mode message to the motor, with a header of motor_id and data array of data

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
        """
        DLC = len(data)
        assert (DLC <= 8), ('Data too long in message for motor ' + str(motor_id))
        
        if self.debug:
            print('ID: ' + str(hex(motor_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
        
        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info) )
        except can.CanError:
            if self.debug:
                print("    Message NOT sent")

    # send the power on code
    def power_on(self, motor_id):
        """
        Sends the power on code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC])
        
    # send the power off code
    def power_off(self, motor_id):
        """
        Sends the power off code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD])

    # send the zeroing code. Like a scale, it takes about a second to zero the position
    def zero(self, motor_id):
        """
        Sends the zeroing code to motor_id. This code will shut off communication with the motor for about a second.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    # send an MIT control signal, consisting of desired position, velocity, and current, and gains for position and velocity control
    # basically an impedance controller
    def MIT_controller(self, motor_id, motor_type, position, velocity, Kp, Kd, I):
        """
        Sends an MIT style control signal to the motor. This signal will be used to generate a 
        current for the field-oriented controller on the motor control chip, given by this expression:

            q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I

        Args:
            motor_id: The CAN ID of the motor to send the message to
            motor_type: A string noting the type of motor, ie 'AK80-9'
            position: The desired position in rad
            velocity: The desired velocity in rad/s
            Kp: The position gain
            Kd: The velocity gain
            I: The additional current
        """
        position_uint16 = CAN_Manager.float_to_uint(position, MIT_Params[motor_type]['P_min'], 
                                                    MIT_Params[motor_type]['P_max'], 16)
        velocity_uint12 = CAN_Manager.float_to_uint(velocity, MIT_Params[motor_type]['V_min'], 
                                                    MIT_Params[motor_type]['V_max'], 12)
        Kp_uint12 = CAN_Manager.float_to_uint(Kp, MIT_Params[motor_type]['Kp_min'], 
                                                    MIT_Params[motor_type]['Kp_max'], 12)
        Kd_uint12 = CAN_Manager.float_to_uint(Kd, MIT_Params[motor_type]['Kd_min'], 
                                                    MIT_Params[motor_type]['Kd_max'], 12)
        I_uint12 = CAN_Manager.float_to_uint(I, MIT_Params[motor_type]['I_min'], 
                                                    MIT_Params[motor_type]['I_max'], 12)

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            ((velocity_uint12&0x00F)<<4) | (Kp_uint12) >> 8,
            (Kp_uint12&0x0FF),
            (Kd_uint12) >> 4,
            ((Kd_uint12&0x00F)<<4) | (I_uint12) >> 8,
            (I_uint12&0x0FF)
        ]
        # print(data)
        self.send_MIT_message(motor_id, data)
        
    # convert data recieved from motor in byte format back into floating point numbers in real units
    def parse_MIT_message(self, data, motor_type):
        """
        Takes a RAW MIT message and formats it into readable floating point numbers.

        Args:
            data: the bytes of data from a python-can message object to be parsed
            motor_type: A string noting the type of motor, ie 'AK80-9'

        Returns:
            An MIT_Motor_State namedtuple that contains floating point values for the 
            position, velocity, current, temperature, and error in rad, rad/s, amp, and *C.
            0 means no error.
        """
        assert len(data) == 8 or len(data) == 6, 'Tried to parse a CAN message that was not Motor State in MIT Mode'
        temp = None
        error = None
        position_uint = data[1] <<8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4]>>4) <<4 ) >> 4
        current_uint = (data[4]&0x0F)<<8 | data[5]
        
        if len(data)  == 8:
            temp = int(data[6])
            error = int(data[7])

        position = CAN_Manager.uint_to_float(position_uint, MIT_Params[motor_type]['P_min'], 
                                            MIT_Params[motor_type]['P_max'], 16)
        velocity = CAN_Manager.uint_to_float(velocity_uint, MIT_Params[motor_type]['V_min'], 
                                            MIT_Params[motor_type]['V_max'], 12)
        current = CAN_Manager.uint_to_float(current_uint, MIT_Params[motor_type]['I_min'], 
                                            MIT_Params[motor_type]['I_max'], 12)

        if self.debug:
            print('  Position: ' + str(position))
            print('  Velocity: ' + str(velocity))
            print('  Current: ' + str(current))
            if (temp is not None) and (error is not None):
                print('  Temp: ' + str(temp))
                print('  Error: ' + str(error))

        return MIT_motor_state(position, velocity, current, temp, error)
