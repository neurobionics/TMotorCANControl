import can
import time
import os
import csv
import traceback
from collections import namedtuple
from enum import Enum
from math import isfinite
import StatProfiler
from SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import warnings

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
            'NM_PER_AMP': 0.146, # probably the same if its the same motor
            'GEAR_RATIO': 9.0 # hence the 9 in the name
        }
}


# Data structure to store and update motor states
class motor_state:
    def __init__(self,position, velocity, current, temperature, error, acceleration):
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(self, position, velocity, current, temperature, error, acceleration):
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state):
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration

# data structure to store MIT_command that will be sent upon update
class MIT_command:
    def __init__(self, position, velocity, kp, kd, current):
        self.position = position
        self.velocity = velocity
        self.kp = kp
        self.kd = kd
        self.current = current

# motor state from the controller, uneditable named tuple
MIT_motor_state = namedtuple('motor_state', 'position velocity current temperature error')

# python-can listener object, with handler to be called upon reception of a message on the CAN bus
class motorListener(can.Listener):
    def __init__(self, _canman, motor):
        self._canman = _canman
        self.bus = _canman.bus
        self.motor = motor

    def on_message_received(self, msg):
            data = bytes(msg.data)
            ID = data[0]
            if ID == self.motor.ID:
                self.motor._update_state_async(self._canman.parse_MIT_message(data, self.motor.type))
            

# A class to manage the low level CAN communication protocols
class CAN_Manager(object):
    
    debug = False
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    def __new__(cls):
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            print("Initializing CAN Manager")
            # verify the CAN bus is currently down
            os.system( 'sudo /sbin/ip link set can0 down' )
            # start the CAN bus back up
            os.system( 'sudo /sbin/ip link set can0 up type can bitrate 1000000' )
            # create a python-can bus object
            cls._instance.bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
            # create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self):
        pass
        
    def __del__(self):
        # shut down the CAN bus when the object is deleted
        # I think this may not ever get called, so keep a pointer and explicitly delete if this is important.
        os.system( 'sudo /sbin/ip link set can0 down' ) 

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor(self, motor):
        self.notifier.add_listener(motorListener(self, motor))


    # Locks value between min and max
    @staticmethod
    def limit_value(value, min, max):
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
        x = CAN_Manager.limit_value(x,x_min,x_max)
        span = x_max-x_min
        # (x - x_min)*(2^num_bits)/span
        return int((x- x_min)*( float((1<<num_bits)/span)) )

    # undoes the above method
    @staticmethod
    def uint_to_float(x,x_min,x_max,num_bits):
        span = x_max-x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x*span/((1<<num_bits)-1) + x_min)

    # sends a message to the motor (when the motor is in MIT mode)
    def send_MIT_message(self, motor_id, data):
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
        self.send_MIT_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC])
        
    # send the power off code
    def power_off(self, motor_id):
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD])

    # send the zeroing code. Like a scale, it takes about a second to zero the position
    def zero(self, motor_id):
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    # send an MIT control signal, consisting of desired position, velocity, and current, and gains for position and velocity control
    # basically an impedance controller
    def MIT_controller(self, motor_id, motor_type, position, velocity, Kp, Kd, I):
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


# defualt variables to be logged
LOG_VARIABLES = [
    "output_angle", 
    "output_velocity", 
    "output_acceleration", 
    "current",
    "output_torque"
]

# possible states for the controller
class _TMotorManState(Enum):
    IDLE = 0
    IMPEDANCE = 1
    CURRENT = 2
    FULL_STATE = 3


# the user-facing class that manages the motor.
class TMotorManager():
    
    def __init__(self, motor_type='AK80-9', motor_ID=1, CSV_file=None, log_vars = LOG_VARIABLES):
        
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        print("Initializing device: " + self.device_info_string())

        self._motor_state = motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._motor_state_async = motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._command = MIT_command(0.0,0.0,0.0,0.0,0.0)
        self._control_state = _TMotorManState.IDLE
        self._times_past_limit = 0
        self._angle_threshold = MIT_Params[self.type]['P_max'] - 2.0 # radians, only really matters if the motor's going super fast
        self._old_pos = 0.0

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False
        
        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "output_angle" : self.get_output_angle_radians, 
            "output_velocity" : self.get_output_velocity_radians_per_second, 
            "output_acceleration" : self.get_output_acceleration_radians_per_second_squared, 
            "current" : self.get_current_qaxis_amps,
            "output_torque": self.get_output_torque_newton_meters,
            "motor_angle" : self.get_motor_angle_radians, 
            "motor_velocity" : self.get_motor_velocity_radians_per_second, 
            "motor_acceleration" : self.get_motor_acceleration_radians_per_second_squared, 
            "motor_torque": self.get_motor_torque_newton_meters 
        }
        
        self._canman = CAN_Manager()
        self._canman.add_motor(self)
        

    def __enter__(self):
        print('Turning on control for device: ' + self.device_info_string())
        if self.csv_file_name is not None:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.log_vars)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        self.power_on()
        self._send_command()
        self._entered = True
        return self

    def __exit__(self, etype, value, tb):
        print('Turning off control for device: ' + self.device_info_string())
        self.power_off()

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    # this method is called by the handler every time a message is recieved on the bus
    # from this motor, to store the most recent state information for later
    def _update_state_async(self, MIT_state):
        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        acceleration = (MIT_state.velocity - self._motor_state_async.velocity)/dt

        self._motor_state_async.set_state(MIT_state.position, MIT_state.velocity, MIT_state.current, MIT_state.temperature, MIT_state.error, acceleration)
        self._updated = True

    # this method is called by the user to synchronize the current state used by the controller
    # with the most recent message recieved
    def update(self):

        # check that the motor is safely turned on
        if not self._entered:
            raise RuntimeError("Tried to update motor state before safely powering on for device: " + self.device_info_string())

        # check that the motor data is recent
        # print(self._command_sent)
        now = time.time()
        if (now - self._last_command_time) < 0.25 and ( (now - self._last_update_time) > 0.1):
            # print("State update requested but no data recieved from motor. Delay longer after zeroing, decrease frequency, or check connection.")
            warnings.warn("State update requested but no data from motor. Delay longer after zeroing, decrease frequency, or check connection. " + self.device_info_string(), RuntimeWarning)
        else:
            self._command_sent = False
        # artificially extending the range of the position that we track
        P_max = MIT_Params[self.type]['P_max']
        old_pos = self._old_pos
        new_pos = self._motor_state_async.position
        thresh = self._angle_threshold
        if (thresh <= new_pos and new_pos <= P_max) and (-P_max <= old_pos and old_pos <= -thresh):
            self._times_past_limit -= 1
        elif (thresh <= old_pos and old_pos <= P_max) and (-P_max <= new_pos and new_pos <= -thresh) :
            self._times_past_limit += 1
            
        # update position
        self._old_pos = new_pos
        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position += self._times_past_limit*2*P_max
        
        # send current motor command
        self._send_command()

        # writing to log file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([self._last_update_time - self._start_time] + [self.LOG_FUNCTIONS[var]() for var in self.log_vars])

        self._updated = False
        
        

    # sends a command to the motor depending on whats controlm mode the motor is in
    def _send_command(self):
        if self._control_state == _TMotorManState.FULL_STATE:
            self._canman.MIT_controller(self.ID,self.type, self._command.position, self._command.velocity, self._command.kp, self._command.kd, self._command.current)
        elif self._control_state == _TMotorManState.IMPEDANCE:
            self._canman.MIT_controller(self.ID,self.type, self._command.position, self._command.velocity, self._command.kp, self._command.kd, 0.0)
        elif self._control_state == _TMotorManState.CURRENT:
            self._canman.MIT_controller(self.ID, self.type, 0.0, 0.0, 0.0, 0.0, self._command.current)
        elif self._control_state == _TMotorManState.IDLE:
            self._canman.MIT_controller(self.ID,self.type, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            raise RuntimeError("UNDEFINED STATE for device " + self.device_info_string())
        self._last_command_time = time.time()

    # Basic Motor Utility Commands
    def power_on(self):
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self):
        self._canman.power_off(self.ID)

    # zeros the position, like a scale you have to wait about a second before you can
    # use the motor again. This responsibility is on the user!!
    def zero_position(self):
        self._canman.zero(self.ID)
        self._last_command_time = time.time()

    # getters for motor state
    def get_current_qaxis_amps(self):
        return self._motor_state.current

    def get_output_angle_radians(self):
        return self._motor_state.position

    def get_output_velocity_radians_per_second(self):
        return self._motor_state.velocity

    def get_output_acceleration_radians_per_second_squared(self):
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self):
        return self.get_current_qaxis_amps()*MIT_Params[self.type]["NM_PER_AMP"]*MIT_Params[self.type]["GEAR_RATIO"]

    # not implemented but in other version
    def set_position_gains(self, kp=200, ki=50, kd=0):
        raise NotImplemented()

    # uses plain impedance mode, will send 0.0 for current command.
    def set_impedance_gains_real_unit(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        assert(isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"])
        assert(isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"])
        self._command.kp = K
        self._command.kd = B
        self._control_state = _TMotorManState.IMPEDANCE
    
    # uses full MIT mode, will send whatever current command is set. 
    def set_impedance_gains_real_unit_full_state_feedback(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        assert(isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"])
        assert(isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"])
        self._command.kp = K
        self._command.kd = B
        self._control_state = _TMotorManState.FULL_STATE

    # uses plain current mode, will send 0.0 for position gains.
    def set_current_gains(self, kp=40, ki=400, ff=128, spoof=False):
        self._control_state = _TMotorManState.CURRENT

    # uses full MIT mode, will send whatever position gains and values are set
    def set_current_gains_full_state_feedback(self, kp=40, ki=400, ff=128, spoof=False):
        self._control_state = _TMotorManState.FULL_STATE

    # used for either impedance or MIT mode to set output angle
    def set_output_angle_radians(self, pos):
        # position commands must be within a certain range :/
        # pos = (np.abs(pos) % MIT_Params[self.type]["P_max"])*np.sign(pos) # this doesn't work because it will unwind itself!
        # CANNOT Control using impedance mode for angles greater than 12.5 rad!!
        if np.abs(pos) >= MIT_Params[self.type]["P_max"]:
            raise RuntimeError("Cannot control using impedance mode for angles with magnitude greater than " + str(MIT_Params[self.type]["P_max"]) + "rad!")

        if self._control_state not in [_TMotorManState.IMPEDANCE, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send position command without gains for device " + self.device_info_string()) 
        self._command.position = pos

    # used for either current MIT mode to set current
    def set_motor_current_qaxis_amps(self, current):
        if self._control_state not in [_TMotorManState.CURRENT, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send current command before entering current mode for device " + self.device_info_string()) 
        self._command.current = current

    # used for either current or MIT Mode to set current, based on desired torque
    def set_output_torque_newton_meters(self, torque):
        self.set_motor_current_qaxis_amps((torque/MIT_Params[self.type]["NM_PER_AMP"]/MIT_Params[self.type]["GEAR_RATIO"]) )

    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque):
        self.set_motor_current_qaxis_amps(torque/(MIT_Params[self.type]["NM_PER_AMP"]) )

    def set_motor_angle_radians(self, pos):
        self.set_output_angle_radians(pos/(MIT_Params[self.type]["GEAR_RATIO"]) )

    def get_motor_angle_radians(self):
        return self._motor_state.position*MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self):
        return self._motor_state.velocity*MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self):
        return self._motor_state.acceleration*MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self):
        return self.get_current_qaxis_amps()*MIT_Params[self.type]["NM_PER_AMP"]

    # Pretty stuff
    def __str__(self):
        return self.device_info_string() + " | Position: " + str(round(self.θ,3)) + " rad | Velocity: " + str(round(self.θd ,3)) + " rad/s | current: " + str(round(self.i,3)) + " A | torque: " + str(round(self.τ,3)) + " Nm"

    def device_info_string(self):
        return str(self.type) + "  ID: " + str(self.ID)


    ## Greek letter math symbol property interface. This is the good
    #  interface, for those who like code that resembles math. It works best
    #  to use the UnicodeMath plugin for sublime-text, "Fast Unicode Math
    #  Characters" in VS Code, or the like to allow easy typing of ϕ, θ, and
    #  τ.

    # electrical variables
    i = property(get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps_current_only")

    # output-side variables
    θ = property(get_output_angle_radians, set_output_angle_radians, doc="output_angle_radians_impedance_only")
    θd = property (get_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, doc="output_acceleration_radians_per_second_squared")
    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters, doc="output_torque_newton_meters")

    # motor-side variables
    ϕ = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians_impedance_only")
    ϕd = property (get_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    ϕdd = property(get_motor_acceleration_radians_per_second_squared, doc="motor_acceleration_radians_per_second_squared")
    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters, doc="motor_torque_newton_meters")
    

# A sample program--should do nothing, then oscillate, then oscillate wider
if __name__ == "__main__":
    # use the with block to safely shut down
    with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as motor3:
        # zero the position
        motor3.zero_position()

        # wait to ensure the motor is zeroed before sending commands
        time.sleep(1.2)

        # set the gains for our controller, enter impedance only mode
        motor3.set_impedance_gains_real_unit(K=10,B=0.5)
        
        # create a soft realtime loop to ensure steady timing
        loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
        for t in loop:
            # update motor state and send the current command (idle to start)
            motor3.update()
            if t < 1.0:
                motor3.θ = 0.0
            elif t < 4:
                motor3.θ = 0.5*np.sin(np.pi*t)
            else:
                motor3.θ = 2*np.sin(0.5*np.pi*t)

        # ensure the loop's destructor is called explicitly to show timing data
        del loop

        


        






