from TMotorCANControl.CAN_manager_servo import *
import can
import time
import csv
import traceback
from collections import namedtuple
from enum import Enum
from math import isfinite
import numpy as np
import warnings


# default variables to be logged
LOG_VARIABLES = [
        "motor_position" , 
        "motor_speed" , 
        "motor_current", 
        "motor_temperature" 
]

# possible states for the controller
class _TMotorManState_Servo(Enum):
    """
    An Enum to keep track of different control states
    """
    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    SET_ORIGIN=5
    POSITION_VELOCITY=6


# the user-facing class that manages the motor.
class TMotorManager_servo():
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """
    def __init__(self, motor_type='AK80-9', motor_ID=1, CSV_file=None, log_vars = LOG_VARIABLES, use_torque_compensation=False):
        """
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list. The full list of possibilities is
            - "output_angle"
            - "output_velocity"
            - "output_acceleration"
            - "current"
            - "output_torque"
            - "motor_angle"
            - "motor_velocity"
            - "motor_acceleration"
            - "motor_torque"
            use_torque_compensation: Enables a more complex torque model to compensate for friction, if available
        """
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        print("Initializing device: " + self.device_info_string())

        self._motor_state = servo_motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._motor_state_async = servo_motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._command = servo_command(0.0,0.0,0.0,0.0)
        self._control_state = _TMotorManState_Servo.DUTY_CYCLE
        self._times_past_position_limit = 0
        self._times_past_current_limit = 0
        self._times_past_velocity_limit = 0
        self._angle_threshold = Servo_Params[self.type]['P_max']/10 - 2.0 # radians, only really matters if the motor's going super fast
        self._current_threshold = self.TMotor_current_to_qaxis_current(Servo_Params[self.type]['T_max']) - 3.0 # A, only really matters if the current changes quick
        self._velocity_threshold = Servo_Params[self.type]['V_max']/0.01 - 2.0 # radians, only really matters if the motor's going super fast
        self._old_pos = None
        self._old_curr = 0.0
        self._old_vel = 0.0
        self._old_current_zone = 0

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False
        self.use_torque_compensation = use_torque_compensation
        self.SF = 1.0
        self.extra_plots = []
        
        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "motor_position" : 0, 
            "motor_speed" : 0, 
            "motor_current" : 0, 
            "motor_temperature" : 0,
        }
        
        self._canman = CAN_Manager_servo()
        self._canman.add_motor(self)
        
            
    def __enter__(self):
        """
        Used to safely power the motor on and begin the log file.
        """
        print('Turning on control for device: ' + self.device_info_string())
        if self.csv_file_name is not None:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.log_vars)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)
        self.power_on() #TODO: How to control this?
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError("Device not connected: " + str(self.device_info_string()))
        return self

    def __exit__(self, etype, value, tb):
        """
        Used to safely power the motor off and close the log file.
        """
        print('Turning off control for device: ' + self.device_info_string())
        self.power_off()#TODO: How to control this

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)
    def TMotor_current_to_qaxis_current(self, iTM):
        return Servo_Params[self.type]['Current_Factor']*iTM/(Servo_Params[self.type]['GEAR_RATIO']*Servo_Params[self.type]['Kt_TMotor'])
    
    def qaxis_current_to_TMotor_current(self, iq):
        return iq*(Servo_Params[self.type]['GEAR_RATIO']*Servo_Params[self.type]['Kt_TMotor'])/Servo_Params[self.type]['Current_Factor']

    # this method is called by the handler every time a message is recieved on the bus
    # from this motor, to store the most recent state information for later
    def _update_state_async(self, servo_state):
        """
        This method is called by the handler every time a message is recieved on the bus
        from this motor, to store the most recent state information for later
        
        Args:
            MIT_state: The MIT_Motor_State namedtuple with the most recent motor state.

        Raises:
            RuntimeError when device sends back an error code that is not 0 (0 meaning no error)
        """
        if servo_state.error != 0:
            raise RuntimeError('Driver board error for device: ' + self.device_info_string() + ": " + Servo_Params['ERROR_CODES'][servo_state.error])

        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        acceleration = (servo_state.velocity - self._motor_state_async.velocity)/dt

        # The "Current" supplied by the controller is actually current*Kt, which approximates torque.
        self._motor_state_async.set_state(servo_state.position, servo_state.velocity, self.TMotor_current_to_qaxis_current(servo_state.current),servo_state.temperature,servo_state.error, acceleration)
        
        self._updated = True

    
    # this method is called by the user to synchronize the current state used by the controller
    # with the most recent message recieved
    def update(self):
        """
        This method is called by the user to synchronize the current state used by the controller
        with the most recent message recieved, as well as to send the current command.
        """

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

        
        if self._old_pos is None:
            self._old_pos = self._motor_state_async.position
        
        new_pos = self._motor_state_async.position
        new_curr = self._motor_state_async.current
        new_vel = self._motor_state_async.velocity

        # update expanded state variables
        self._old_pos = new_pos
        self._old_curr = new_curr
        self._old_vel = new_vel

        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position += self._times_past_position_limit*2*Servo_Params[self.type]['P_max']/10
        self._motor_state.current = new_curr
        self._motor_state.velocity += self._times_past_velocity_limit*2*Servo_Params[self.type]['V_max']/0.01
        
        # send current motor command
        self._send_command()

        # writing to log file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([self._last_update_time - self._start_time] + [self.LOG_FUNCTIONS[var]() for var in self.log_vars] + self._times_past_current_limit +[data for data in self.extra_plots])

        self._updated = False
    # sends a command to the motor depending on whats controlm mode the motor is in
    def _send_command(self):
        """
        Sends a command to the motor depending on whats controlm mode the motor is in. This method
        is called by update(), and should only be called on its own if you don't want to update the motor state info.

        Notably, the current is converted to amps from the reported 'torque' value, which is i*Kt. 
        This allows control based on actual q-axis current, rather than estimated torque, which 
        doesn't account for friction losses.
        """
        if self._control_state == _TMotorManState_Servo.DUTY_CYCLE:
            self._canman.comm_can_set_duty(self.ID,self._command.duty)
        elif self._control_state == _TMotorManState_Servo.CURRENT_LOOP:
            self._canman.comm_can_set_current(self.ID,self._command.current)
        elif self._control_state == _TMotorManState_Servo.CURRENT_BRAKE:
            self._canman.comm_can_set_cb(self.ID,self._command.current)
        elif self._control_state == _TMotorManState_Servo.VELOCITY:
            self._canman.comm_can_set_rpm(self.ID, self._command.velocity)
        elif self._control_state == _TMotorManState_Servo.POSITION:
            self._canman.comm_can_set_rpm(self.ID, self._command.position)


        #TODO:Add other modes
        else:
            raise RuntimeError("UNDEFINED STATE for device " + self.device_info_string())
        self._last_command_time = time.time()

    # Basic Motor Utility Commands
    def power_on(self):
        """Powers on the motor. You may hear a faint hiss."""
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self):
        """Powers off the motor."""
        self._canman.power_off(self.ID)

    # zeros the position, like a scale you have to wait about a second before you can
    # use the motor again. This responsibility is on the user!!
    def zero_position(self):
        """Zeros the position--like a scale you have to wait about a second before you can
        use the motor again. This responsibility is on the user!!"""
        self._canman.zero(self.ID)
        self._last_command_time = time.time()

    # getters for motor state
    def get_temperature_celsius(self):
        """
        Returns:
        The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.temperature
    
    def get_motor_error_code(self):
        """
        Returns:
        The most recently updated motor error code.
        Note the program should throw a runtime error before you get a chance to read
        this value if it is ever anything besides 0.

        Codes:
        - 0 : 'No Error',
        - 1 : 'Over temperature fault',
        - 2 : 'Over current fault',
        - 3 : 'Over voltage fault',
        - 4 : 'Under voltage fault',
        - 5 : 'Encoder fault',
        - 6 : 'Phase current unbalance fault (The hardware may be damaged)'
        """
        return self._motor_state.error

    def get_current_qaxis_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.current

    def get_output_angle_radians(self):
        """
        Returns:
        The most recently updated output angle in radians
        """
        return self._motor_state.position

    def get_output_velocity_radians_per_second(self):
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.velocity

    def get_output_acceleration_radians_per_second_squared(self):
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self):
        """
        Returns:
            the most recently updated output torque in Nm
        """
        if Servo_Params[self.type]['Use_derived_torque_constants'] and self.use_torque_compensation:
            a_hat = Servo_Params[self.type]['a_hat']
            kt = Servo_Params[self.type]["Kt_actual"]
            gr = Servo_Params[self.type]["GEAR_RATIO"]
            ϵ = 0.1
            i = self.get_current_qaxis_amps()
            v = self.get_motor_velocity_radians_per_second()

            return a_hat[0] + a_hat[1]*gr*kt*i - a_hat[2]*gr*np.abs(i)*i - a_hat[3]*np.sign(v)*(np.abs(v)/(ϵ + np.abs(v)) ) - a_hat[4]*np.abs(i)*np.sign(v)*(np.abs(v)/(ϵ + np.abs(v)) )
        else:
            return self.get_current_qaxis_amps()*Servo_Params[self.type]["Kt_actual"]*Servo_Params[self.type]["GEAR_RATIO"]



    # used for either impedance or MIT mode to set output angle
    def set_output_angle_radians(self, pos):
        """
        Used for either impedance or full state feedback mode to set output angle command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            pos: The desired output position in rads
        """
        # position commands must be within a certain range :/
        # pos = (np.abs(pos) % MIT_Params[self.type]["P_max"])*np.sign(pos) # this doesn't work because it will unwind itself!
        # CANNOT Control using impedance mode for angles greater than 12.5 rad!!
        if np.abs(pos) >= Servo_Params[self.type]["P_max"]:
            raise RuntimeError("Cannot control using impedance mode for angles with magnitude greater than " + str(Servo_Params[self.type]["P_max"]) + "rad!")
        self._command.position = pos

    def set_output_velocity_radians_per_second(self, vel):
        """
        Used for either speed or full state feedback mode to set output velocity command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(vel) >= Servo_Params[self.type]["V_max"]:
            raise RuntimeError("Cannot control using speed mode for angles with magnitude greater than " + str(Servo_Params[self.type]["V_max"]) + "rad/s!")

        if self._control_state not in [_TMotorManState_Servo.VELOCITY]:
            raise RuntimeError("Attempted to send speed command without gains for device " + self.device_info_string()) 
        self._command.velocity = vel

    # used for either current MIT mode to set current
    def set_motor_current_qaxis_amps(self, current):
        """
        Used for either current or full state feedback mode to set current command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.
        
        Args:
            current: the desired current in amps.
        """
        if self._control_state not in [_TMotorManState_Servo.CURRENT_LOOP, _TMotorManState_Servo.DUTY_CYCLE]:
            raise RuntimeError("Attempted to send current command before entering current mode for device " + self.device_info_string()) 
        self._command.current = current

    # used for either current or MIT Mode to set current, based on desired torque
    def set_output_torque_newton_meters(self, torque):
        """
        Used for either current or MIT Mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.
        
        Args:
            torque: The desired output torque in Nm.
        """
        if Servo_Params[self.type]['Use_derived_torque_constants'] and self.use_torque_compensation:
            a_hat = Servo_Params[self.type]['a_hat']
            kt = Servo_Params[self.type]["Kt_actual"]
            gr = Servo_Params[self.type]["GEAR_RATIO"]
            ϵ = 0.1
            i = self.get_current_qaxis_amps()
            v = self.get_motor_velocity_radians_per_second()
            bias = - a_hat[0]
            friction = self.SF*(v/(ϵ + np.abs(v)) )*(a_hat[3] + a_hat[4]*np.abs(i)) 
            torque_constant = (gr*(a_hat[1]*kt - a_hat[2]*np.abs(i)))
            Iq_des = (torque - bias + friction )/torque_constant
            self.set_motor_current_qaxis_amps(Iq_des)
        else:
            self.set_motor_current_qaxis_amps((torque/Servo_Params[self.type]["Kt_actual"]/Servo_Params[self.type]["GEAR_RATIO"]) )



    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque):
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque
        
        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque_newton_meters(torque*Servo_Params[self.type]["Kt_actual"])

    def set_motor_angle_radians(self, pos):
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle
        
        Args:
            pos: The desired motor-side position in rad.
        """
        self.set_output_angle_radians(pos/(Servo_Params[self.type]["GEAR_RATIO"]) )

    def set_motor_velocity_radians_per_second(self, vel):
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity
        
        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity_radians_per_second(vel/(Servo_Params[self.type]["GEAR_RATIO"]) )

    def get_motor_angle_radians(self):
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle
        
        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position*Servo_Params[self.type]["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self):
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity
        
        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity*Servo_Params[self.type]["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self):
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration
        
        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration*Servo_Params[self.type]["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self):
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque
        
        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.get_output_torque_newton_meters()*Servo_Params[self.type]["GEAR_RATIO"]

    # Pretty stuff
    def __str__(self):
        """Prints the motor's device info and current"""
        return self.device_info_string() + " | Position: " + '{: 1f}'.format(round(self.θ,3)) + " rad | Velocity: " + '{: 1f}'.format(round(self.θd,3)) + " rad/s | current: " + '{: 1f}'.format(round(self.i,3)) + " A | torque: " + '{: 1f}'.format(round(self.τ,3)) + " Nm"

    def device_info_string(self):
        """Prints the motor's ID and device type."""
        return str(self.type) + "  ID: " + str(self.ID)

    # Checks the motor connection by sending a 10 commands and making sure the motor responds.
    def check_can_connection(self):
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.
        """
        if not self._entered:
            raise RuntimeError("Tried to check_can_connection before entering motor control! Enter control using the __enter__ method, or instantiating the TMotorManager in a with block.")
        Listener = can.BufferedReader()
        self._canman.notifier.add_listener(Listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        success = True
        time.sleep(0.1)
        for i in range(10):
            if Listener.get_message(timeout=0.1) is None:
                success = False
        self._canman.notifier.remove_listener(Listener)
        return success

   



    ## Greek letter math symbol property interface. This is the good
    #  interface, for those who like code that resembles math. It works best
    #  to use the UnicodeMath plugin for sublime-text, "Fast Unicode Math
    #  Characters" in VS Code, or the like to allow easy typing of ϕ, θ, and
    #  τ.

    # controller variables
    T = property(get_temperature_celsius, doc="temperature_degrees_C")
    """Temperature in Degrees Celsius"""

    e = property(get_motor_error_code, doc="temperature_degrees_C")
    """Motor error code. 0 means no error."""

    # electrical variables
    i = property(get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps_current_only")
    """Q-axis current in amps"""

    # output-side variables
    θ = property(get_output_angle_radians, set_output_angle_radians, doc="output_angle_radians_impedance_only")
    """Output angle in rad"""

    θd = property (get_output_velocity_radians_per_second, set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    """Output velocity in rad/s"""

    θdd = property(get_output_acceleration_radians_per_second_squared, doc="output_acceleration_radians_per_second_squared")
    """Output acceleration in rad/s/s"""

    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters, doc="output_torque_newton_meters")
    """Output torque in Nm"""

    # motor-side variables
    ϕ = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians_impedance_only")
    """Motor-side angle in rad"""
    
    ϕd = property (get_motor_velocity_radians_per_second, set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    """Motor-side velocity in rad/s"""

    ϕdd = property(get_motor_acceleration_radians_per_second_squared, doc="motor_acceleration_radians_per_second_squared")
    """Motor-side acceleration in rad/s/s"""

    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters, doc="motor_torque_newton_meters")
    """Motor-side torque in Nm"""


