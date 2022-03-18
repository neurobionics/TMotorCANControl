""" An object that wraps the Dephy ActPack """

import os, sys
import time
import csv
import traceback
import numpy as np
import h5py
import deprecated
from enum import Enum
from math import isfinite

# Dephy library import
from flexsea import fxUtils as fxu  # pylint: disable=no-name-in-module
from flexsea import fxEnums as fxe  # pylint: disable=no-name-in-module
from flexsea import flexsea as flex

# Version of the ActPackMan library
__version__="1.0.0"

class FlexSEA(flex.FlexSEA):
    """ A singleton class that prevents re-initialization of FlexSEA """
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            print("making a new one")
            cls._instance = flex.FlexSEA()
        return cls._instance

    def __init__(self):
        pass


# See ActPackState for all available data
labels = [ # matches varsToStream
    "State time",  
    "Motor angle", "Motor velocity", "Motor acceleration",  
    "Motor voltage", "Motor current",   
    "Battery voltage", "Battery current"
]
DEFAULT_VARIABLES = [ # struct fields defined in flexsea/dev_spec/ActPackState.py
    "state_time",
    "mot_ang", "mot_vel", "mot_acc",
    "mot_volt", "mot_cur", 
    "batt_volt", "batt_curr"
]



MOTOR_CLICKS_PER_REVOLUTION = 16384 
RAD_PER_SEC_PER_GYRO_LSB = np.pi/180/32.8
G_PER_ACCELEROMETER_LSB = 1./8192
NM_PER_AMP = 0.146
RAD_PER_CLICK = 2*np.pi/MOTOR_CLICKS_PER_REVOLUTION
RAD_PER_DEG = np.pi/180.
ticks_to_motor_radians = lambda x: x*(np.pi/180./45.5111)
motor_radians_to_ticks = lambda q: q*(180*45.5111/np.pi)

class _ActPackManStates(Enum):
    VOLTAGE = 1
    CURRENT = 2
    POSITION = 3
    IMPEDANCE = 4


class ActPackMan(object):
    """ (Dephy) Actuator Pack Manager
    Keeps track of a single Dephy Actuator
    """
    
    def __init__(self, devttyACMport, baudRate=230400, csv_file_name=None,
        hdf5_file_name=None, vars_to_log=DEFAULT_VARIABLES, gear_ratio=1.0,
        printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6):
        """ Intializes variables, but does not open the stream. """

        #init printer settings
        self.updateFreq = updateFreq
        self.shouldLog = shouldLog
        self.logLevel = logLevel
        self.prevReadTime = time.time()-1/self.updateFreq
        self.gear_ratio = gear_ratio

        # self.varsToStream = varsToStream
        self.baudRate = baudRate
        self.devttyACMport = devttyACMport
        self.csv_file_name = csv_file_name
        self.hdf5_file_name = hdf5_file_name
        self.csv_file = None
        self.csv_writer = None
        self.vars_to_log = vars_to_log
        self.entered = False
        self._state = None
        self.act_pack = None # code for never having updated

    ## 'With'-block interface for ensuring a safe shutdown.

    def __enter__(self):
        """ Runs when the object is used in a 'with' block. Initializes the comms."""
        if self.csv_file_name is not None:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.vars_to_log)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        if self.hdf5_file_name is not None:
            self.hdf5_file = h5py.File(self.hdf5_file_name, 'w')


        fxs = FlexSEA() # grab library singleton (see impl. in ActPackMan.py)
        # dev_id = fxs.open(port, baud_rate, log_level=6)
        self.dev_id = fxs.open(self.devttyACMport, self.baudRate, log_level=self.logLevel)
        # fxs.start_streaming(dev_id, 100, log_en=False)
        print('devID', self.dev_id)
        # Start stream
        fxs.start_streaming(self.dev_id, self.updateFreq, log_en=self.shouldLog)

        # app_type = fxs.get_app_type(dev_id)
        self.app_type = fxs.get_app_type(self.dev_id)
        print(self.app_type)

        self._state = _ActPackManStates.VOLTAGE
        self.entered = True
        return self

    def __exit__(self, etype, value, tb):
        """ Runs when leaving scope of the 'with' block. Properly terminates comms and file access."""

        if not (self.dev_id is None):
            print('Turning off control for device %s'%self.devttyACMport)
            fxs = FlexSEA() # singleton
            fxs.send_motor_command(self.dev_id, fxe.FX_NONE, 0) # 0 mV
            self.v = 0.0
            print('sleeping')
            # fxs.stop_streaming(self.dev_id) # experimental
            # sleep(0.1) # Works
            time.sleep(0.001) # Works
            # sleep(0.0) # doesn't work in that it results in the following ridiculous warning:
                # "Detected stream from a previous session, please power cycle the device before continuing"
            print('done sleeping')
            fxs.close(self.dev_id)
        
        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)


    ## Critical data reading function. Run update exactly once per loop.

    def update(self):
        # fetches new data from the device
        if not self.entered:
            raise RuntimeError("ActPackMan updated before __enter__ (which begins the streaming)")
        currentTime = time.time()
        if abs(currentTime-self.prevReadTime)<0.25/self.updateFreq:
            print("warning: re-updating twice in less than a quarter of a time-step")
        self.act_pack = FlexSEA().read_device(self.dev_id) # a c-types struct
        self.prevReadTime = currentTime

        # Automatically save all the data as a csv file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([time.time()]+[getattr(self.act_pack,x) for x in self.vars_to_log])

        if self.hdf5_file_name is not None:
            raise NotImplemented()

    ## Gain Setting and Control Mode Switching (using hidden member self._state)
    """
    The behavior of these gain-setting function is to require setting gains
    before setting the corresponding set-point. Voltage mode requires no
    gains, and therefore can be accessed at any time. Setting a voltage means
    gains need to be re-specified before any other mode can be controlled.
    """

    def set_position_gains(self, kp=200, ki=50, kd=0):
        assert(isfinite(kp) and 0 <= kp and kp <= 1000)
        assert(isfinite(ki) and 0 <= ki and ki <= 1000)
        assert(isfinite(kd) and 0 <= kd and kd <= 1000)
        self.set_voltage_qaxis_volts(0.0)
        self._state=_ActPackManStates.POSITION
        FlexSEA().set_gains(self.dev_id, kp, ki, kd, 0, 0, 0)
        self.set_motor_angle_radians(self.get_motor_angle_radians())

    def set_current_gains(self, kp=40, ki=400, ff=128, spoof=False):
        """ sets the current gains. """
        print(kp, ki, ff)
        assert(isfinite(kp) and 0 <= kp and kp <= 80)
        assert(isfinite(ki) and 0 <= ki and ki <= 800)
        assert(isfinite(ff) and 0 <= ff and ff <= 128)
        self.set_voltage_qaxis_volts(0.0)
        self._state=_ActPackManStates.CURRENT
        FlexSEA().set_gains(self.dev_id, kp, ki, 0, 0, 0, ff)

        
        self.set_current_qaxis_amps(0.0)

    def set_impedance_gains_raw_unit_KB(self, kp=40, ki=400, K=300, B=1600, ff=128):
        # Use this for integer gains suggested by the dephy website
        assert(isfinite(kp) and 0 <= kp and kp <= 80)
        assert(isfinite(ki) and 0 <= ki and ki <= 800)
        assert(isfinite(ff) and 0 <= ff and ff <= 128)
        assert(isfinite(K) and 0 <= K)
        assert(isfinite(B) and 0 <= B)
        self.set_voltage_qaxis_volts(0.0)
        self._state=_ActPackManStates.IMPEDANCE
        FlexSEA().set_gains(self.dev_id, int(kp), int(ki), 0, int(K), int(B), int(ff))
        self.set_motor_angle_radians(self.get_motor_angle_radians())

    def set_impedance_gains_real_unit_KB(self, kp=40, ki=400, K=0.08922, B=0.0038070, ff=128):
        # This attempts to allow K and B gains to be specified in Nm/rad and Nm s/rad.
        A = 0.00028444
        C = 0.0007812
        Nm_per_rad_to_Kunit = RAD_PER_CLICK/C*1e3/NM_PER_AMP
        Nm_s_per_rad_to_Bunit = RAD_PER_DEG/A*1e3/NM_PER_AMP
        # K_Nm_per_rad = torque_Nm/(RAD_PER_CLICK*delta) = 0.146*1e-3*C*K/RAD_PER_CLICK
        # B_Nm_per_rads = torque_Nm/(vel_deg_sec*RAD_PER_DEG) = 0.146*1e-3*A*B / RAD_PER_DEG
        self.set_impedance_gains_raw_unit_KB(kp=kp, ki=ki, K=K*Nm_per_rad_to_Kunit, B=B*Nm_s_per_rad_to_Bunit, ff=ff)


    ## Primary getters and setters

    # electrical variables

    def get_battery_voltage_volts(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.batt_volt * 1e-3

    def get_battery_current_amps(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.batt_curr * 1e-3

    def get_voltage_qaxis_volts(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_volt * 1e-3

    def set_voltage_qaxis_volts(self, voltage_qaxis):
        self._state = _ActPackManStates.VOLTAGE # gains must be reset after reverting to voltage mode.
        FlexSEA().send_motor_command(self.dev_id, fxe.FX_NONE, int(voltage_qaxis*1000))

    def get_current_qaxis_amps(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_cur * 1e-3

    def set_current_qaxis_amps(self, current_q):
        if self._state != _ActPackManStates.CURRENT:
            raise RuntimeError("Motor must be in current mode to accept a current command")
        FlexSEA().send_motor_command(self.dev_id, fxe.FX_CURRENT, int(current_q*1000.0))


    # motor-side variables

    def get_motor_angle_radians(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return int(self.act_pack.mot_ang)*RAD_PER_CLICK

    def get_motor_velocity_radians_per_second(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_vel*RAD_PER_DEG # expects deg/sec

    def get_motor_acceleration_radians_per_second_squared(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_acc # expects rad/s/s

    def get_motor_torque_newton_meters(self):
        return self.get_current_qaxis_amps()*NM_PER_AMP

    def set_motor_angle_radians(self, pos):
        if self._state not in [_ActPackManStates.POSITION, _ActPackManStates.IMPEDANCE]:
            raise RuntimeError(
                "Motor must be in position or impedance mode to accept a position setpoint")
        FlexSEA().send_motor_command(self.dev_id, fxe.FX_POSITION, int(pos/RAD_PER_CLICK))

    def set_motor_velocity_radians_per_second(self, motor_velocity):
        raise NotImplemented() # potentially a way to specify position, impedance, or voltage commands.

    def set_motor_acceleration_radians_per_second_squared(self, motor_acceleration):
        raise NotImplemented() # potentially a way to specify position, impedance, or current commands.

    def set_motor_torque_newton_meters(self, torque):
        return self.set_current_qaxis_amps(torque/NM_PER_AMP)

    # output variables

    def get_output_angle_radians(self):
        return self.get_motor_angle_radians()/self.gear_ratio

    def get_output_velocity_radians_per_second(self):
        return self.get_motor_velocity_radians_per_second()/self.gear_ratio

    def get_output_acceleration_radians_per_second_squared(self):
        return self.get_motor_acceleration_radians_per_second_squared()/self.gear_ratio

    def get_output_torque_newton_meters(self):
        return self.get_motor_torque_newton_meters()*self.gear_ratio

    def set_output_angle_radians(self, angle):
        self.set_motor_angle_radians(angle*self.gear_ratio)

    def set_output_velocity_radians_per_second(self, vel):
        self.set_motor_velocity_radians_per_second(vel*self.gear_ratio)

    def set_output_acceleration_radians_per_second_squared(self, acc):
        self.set_motor_acceleration_radians_per_second_squared(acc*self.gear_ratio)

    def set_output_torque_newton_meters(self, torque):
        self.set_motor_torque_newton_meters(torque/self.gear_ratio)

    # other
    def get_temp_celsius(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.temp*1.0 # expects Celsius

    def get_gyro_vector_radians_per_second(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return np.array([[1.0*self.act_pack.gyro_x, self.act_pack.gyro_y, self.act_pack.gyro_z]]).T*RAD_PER_SEC_PER_GYRO_LSB# 1.0/32.8 * np.pi/180

    def get_accelerometer_vector_gravity(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return np.array([[self.act_pack.acc_x, self.act_pack.acc_y, self.act_pack.acc_z]]).T*G_PER_ACCELEROMETER_LSB


    ## Greek letter math symbol property interface. This is the good
    #  interface, for those who like code that resembles math. It works best
    #  to use the UnicodeMath plugin for sublime-text, "Fast Unicode Math
    #  Characters" in VS Code, or the like to allow easy typing of ϕ, θ, and
    #  τ.

    # electrical variables
    v = property(get_voltage_qaxis_volts, set_voltage_qaxis_volts, doc="voltage_qaxis_volts")
    i = property(get_current_qaxis_amps, set_current_qaxis_amps, doc="current_qaxis_amps")

    # motor-side variables
    ϕ = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians")
    ϕd = property (get_motor_velocity_radians_per_second, 
        set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    ϕdd = property(get_motor_acceleration_radians_per_second_squared, 
        set_motor_acceleration_radians_per_second_squared, 
        doc="motor_acceleration_radians_per_second_squared")
    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters,
        doc="motor_torque_newton_meters")

    # output-side variables
    θ = property(get_output_angle_radians, set_output_angle_radians)
    θd = property(get_output_velocity_radians_per_second, 
        set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, 
        set_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared")
    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters,
        doc="output_torque_newton_meters")

    # other
    α = property(get_accelerometer_vector_gravity, doc="accelerometer vector, g")
    ω = property(get_gyro_vector_radians_per_second, doc="gyro vector, rad/s")

    ## Weird-unit getters and setters

    def set_motor_angle_clicks(self, pos):
        if self._state not in [_ActPackManStates.POSITION, _ActPackManStates.IMPEDANCE]:
            raise RuntimeError(
                "Motor must be in position or impedance mode to accept a position setpoint")
        FlexSEA().send_motor_command(self.dev_id, fxe.FX_POSITION, int(pos))

    def get_motor_angle_clicks(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_ang

