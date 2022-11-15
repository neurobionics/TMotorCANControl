import serial
from serial import threaded
import time
import numpy as np
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from enum import Enum
import traceback
import csv

Servo_Params_Serial = {
    'AK80-9': {
        'Type' : 'AK80-9', # name of motor to print out in diagnostics
        'P_min' : -58.85, # rad (-58.85 rad limit)
        'P_max' : 58.85, # rad (58.85 rad limit)
        'V_min' : -20.0, # rad/s (-58.18 rad/s limit)
        'V_max' : 20.0, # rad/s (58.18 rad/s limit)
        'Curr_min' : -15.0,# A (-60A is the acutal limit)
        'Curr_max' : 15.0, # A (60A is the acutal limit)
        'Temp_max' : 40.0, # max mosfet temp in deg C
        'Kt': 0.115, # Nm before gearbox per A of qaxis current
        'GEAR_RATIO': 9, # 9:1 gear ratio
        'NUM_POLE_PAIRS' : 21 # 21 pole pairs
    }        
}

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

PARAMETER_FLAGS = {
    # parameter       : flag
    'MOSFET_TEMP'     : 1,      
    'MOTOR_TEMP'      : 1 << 2, 
    'OUTPUT_CURRENT'  : 1 << 3,
    'INPUT_CURRENT'   : 1 << 4,
    'D_CURRENT'       : 1 << 5,
    'Q_CURRENT'       : 1 << 6,
    'DUTY_CYCLE'      : 1 << 7,
    'MOTOR_SPEED'     : 1 << 8,
    'INPUT_VOLTAGE'   : 1 << 9,
    'MOTOR_ERROR_FLAG': 1 << 16,
    'MOTOR_POSITION'  : 1 << 17,
    'MOTOR_ID'        : 1 << 18
}

ERROR_CODES = {
    0  : 'FAULT_CODE_NONE',
    1  : 'FAULT_CODE_OVER_VOLTAGE',
    2  : 'FAULT_CODE_UNDER_VOLTAGE',
    3  : 'FAULT_CODE_DRIVE',
    4  : 'FAULT_CODE_ABS_OVER_CURRENT',
    5  : 'FAULT_CODE_OVER_TEMP_FET',
    6  : 'FAULT_CODE_OVER_TEMP_MOTOR',
    7  : 'FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE',
    8  : 'FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE',
    9  : 'FAULT_CODE_MCU_UNDER_VOLTAGE',
    10 : 'FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET',
    11 : 'FAULT_CODE_ENCODER_SPI',
    12 : 'FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE',
    13 : 'FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE',
    14 : 'FAULT_CODE_FLASH_CORRUPTION',
    15 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1',
    16 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2',
    17 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3',
    18 : 'FAULT_CODE_UNBALANCED_CURRENTS',
}    

class COMM_PACKET_ID():
    COMM_FW_VERSION = 0
    COMM_JUMP_TO_BOOTLOADER = 1
    COMM_ERASE_NEW_APP = 2
    COMM_WRITE_NEW_APP_DATA = 3
    COMM_GET_VALUES = 4
    COMM_SET_DUTY =  5
    COMM_SET_CURRENT = 6
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_POS = 9
    COMM_SET_HANDBRAKE = 10
    COMM_SET_DETECT = 11
    # COMM_GET_POS = 16
    COMM_ROTOR_POSITION = 22
    COMM_GET_VALUES_SETUP = 50
    COMM_SET_POS_SPD = 91
    COMM_SET_POS_MULTI = 92
    COMM_SET_POS_SINGLE = 93
    COMM_SET_POS_UNLIMITED = 94
    COMM_SET_POS_ORIGIN = 95

def buffer_append_int16( buffer,number):
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
def buffer_append_uint16( buffer,number):
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
def buffer_append_int32(buffer,number):
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
def buffer_append_uint32( buffer,number):
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
def buffer_append_int64( buffer,number):
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
def buffer_append_uint64(buffer,number):
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

def buffer_get_int8(data, ind):
    return np.int8((data[ind]))

def buffer_get_int16(data, ind):
    return np.int16((np.uint8(data[ind]) << 8) | np.uint8(data[ind+1]))

def buffer_get_int32(data, ind):
    return np.int32((np.uint8(data[ind]) << 24) | (np.uint8(data[ind+1]) << 16) | (np.uint8(data[ind+2]) << 8) | np.uint8(data[ind+3]))

def crc16(data, DL):
        cksum = np.uint16(0)
        for i in range(DL):
            # From C example code:
            # cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8)
            cksum = crc16_tab[((cksum >> 8) ^ (data[i])) & 0xFF] ^ (cksum << 8)
        return np.uint16(cksum)

def create_frame(data):
    frame = []
    DL = len(data)
    if (DL > 256):
        raise RuntimeError("Tried to send packet longer than 256 bytes!")
    else:
        frame.append(0x02)
        frame.append(DL)
        frame += data
        crc = crc16(data, DL)
        frame.append(np.uint8(crc >> 8))
        frame.append(np.uint8(crc & 0xFF))
        frame.append(0x03)
    return frame

def parse_frame(frame):
    # add error checking later
    print(frame)
    if len(frame) > 4:
        header = frame[0]
        DL = frame[1]
        data = frame[2:2+DL]
        crc = buffer_get_int16(frame[2+DL:DL+4], 0)
        if crc == crc16(data, DL):
            return data
        else:
            return None
    else:
        return None
                
class servo_serial_motor_state:
    def __init__(self):
        self.initialized = False
        self.mos_temperature = 0
        self.motor_temperature = 0
        self.output_current = 0
        self.input_current = 0
        self.id_current = 0
        self.iq_current = 0
        self.duty = 0
        self.speed = 0
        self.input_voltage = 0
        self.position_set = 0
        self.controlID = 0
        self.Vd = 0
        self.Vq = 0
        self.error = 0
        self.acceleration = 0
        self.position = 0

    def set_state(self, mos_temperature = 0,
                motor_temperature = 0,
                output_current = 0,
                input_current = 0,
                id_current = 0,
                iq_current = 0,
                duty = 0,
                speed = 0,
                input_voltage = 0,
                position_set = 0,
                controlID = 0,
                Vd = 0,
                Vq = 0,
                error = 0,
                acceleration = 0,
                position = 0):
        self.initialized = True
        self.mos_temperature = mos_temperature
        self.motor_temperature = motor_temperature 
        self.output_current = output_current
        self.input_current = input_current
        self.id_current = id_current
        self.iq_current = iq_current
        self.duty = duty
        self.speed = speed
        self.input_voltage = input_voltage
        self.position_set = position_set
        self.controlID = controlID
        self.Vd = Vd
        self.Vq = Vq
        self.error = error
        self.acceleration = acceleration
        self.position = position

    
    def __str__(self):
        s = f'Mos Temp: {self.mos_temperature}'
        s += f'\nMotor Temp: {self.motor_temperature}'
        s += f'\nOutput Current: {self.output_current}'
        s += f'\nInput Current: {self.input_current}'
        s += f'\nid current: {self.id_current}'
        s += f'\niq current: {self.iq_current}'
        s += f'\nduty: {self.duty}'
        s += f'\nspeed: {self.speed}'
        s += f'\ninput voltage: {self.input_voltage}'
        s += f'\nposition: {self.position_set}'
        s += f'\ncontrolID: {self.controlID}'
        s += f'\nVd: {self.Vd}'
        s += f'\nVq: {self.Vq}'
        s += f'\nAccel: {self.acceleration}'
        return s

# possible states for the controller
class SERVO_SERIAL_CONTROL_STATE(Enum):
    """
    An Enum to keep track of different control states
    """
    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    HANDBRAKE = 5
    POSITION_VELOCITY = 6
    IDLE = 7


class motor_listener(serial.threaded.Protocol):

    def connection_made(self, transport):
        super().connection_made(transport)

    def connection_lost(self, transport):
        super().connection_made(transport)

    def __init__(self):
        super().__init__()
        self.buffer =[]
        self.state = 0 # 0 : ready, 1 : message started (0x02), 2 : known DL, 3 : ready data, 5 : expect 0x03
        self.DL = 0
        self.i = 0
        self.motor = None

    def data_received(self, data):
        for d in data:
            if self.state == 0:
                if d == 0x02:
                    self.buffer.append(d)
                    self.state = 1
            elif self.state == 1:
                self.buffer.append(d)
                self.DL = d + 2
                self.state = 2
            elif self.state == 2:
                self.buffer.append(d)
                self.i += 1
                if self.i == self.DL:
                    self.state = 3
                    self.i = 0
                    self.DL = 0
            elif self.state == 3:
                if d == 0x03:
                    self.buffer.append(d)
                    self.handle_packet(self.buffer)
                    self.state = 0
                    self.buffer = []
                
    def handle_packet(self, packet):
        if self.motor is not None:
            # print(packet)
            header = packet[0]
            DL = packet[1]
            data = packet[2:2+DL]
            crc = buffer_get_int16(packet[2+DL:DL+4], 0)
            if crc != crc16(data, DL):
                data = None
            if not (data is None) and len(data) > 0:
                self.motor.update_async(data)

# the user-facing class that manages the motor.
class TMotorManager_servo_serial():
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """
    def __init__(self, port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']):
        """
        Add description later
        """
        self.motor_params = motor_params
        self.port = port
        self.baud = baud
        print("Initializing device: " + self.device_info_string())

        self._motor_state = servo_serial_motor_state()
        self._motor_state_async = servo_serial_motor_state()
        self._command = None # overwrite with byte array of command to send on update()
        self._control_state = None

        # TODO verify these work for other motor types!
        self.radps_per_ERPM =2*np.pi/180/60 # 5.82E-04 
        self.rad_per_Eang = 2*(np.pi/180)/(self.motor_params['NUM_POLE_PAIRS']) # 1.85e-4
        
        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._updated_async = False
        self._updated = False
        self._ser = None
        self._reader_thread = None
        
    def __enter__(self):
        """
        Used to safely power the motor on.
        """
        if not self._entered:
            # begin serial connection
            self._ser = serial.Serial(self.port, self.baud)

            # start another thread to handle recieved data
            self._reader_thread = serial.threaded.ReaderThread(self._ser, motor_listener)
            self._listener = self._reader_thread.__enter__() 
            self._listener.motor = self

            # send startup sequence
            self.power_on()
            self.send_command()

            # tell the motor to send back all parameters
            # TODO expand to allow user to only request some data, could be faster
            self.set_motor_parameter_return_format_all()
            self.send_command()

            # tell motor to send current position (for some reason current position is not in the other parameters)
            self.begin_position_feedback()
            self.send_command()

            # don't do anything else
            self.enter_idle_mode()
            self.send_command()

            if not self.check_connection():
                print("device: {self.device_info_string()} not connected!")
            else:
                print(f"device: {self.device_info_string()} successfully connected.")

            self.f = open('timing_test_log.csv','w').__enter__()
            self.w = csv.writer(self.f)
            # return the safely entered motor manager object
            self._entered = True
            return self
        else:
            # it's already been entered!
            print(f"Already entered device: {self.device_info_string()}")
        
    def __exit__(self, etype, value, tb):
        """
        Used to safely power the motor off and close the port.
        """
        print('\nTurning off control for device: ' + self.device_info_string())
        # end reading thread
        self._reader_thread.stop() 

        # power down motor
        self._ser.write(self.set_duty_cycle(0.0)) 

        # end serial connection
        self._ser.close() 

        # if this was ended due to an error, print the stack trace
        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def check_connection(self):
        self._send_specific_command(self.get_motor_parameters())
        self._send_specific_command(self.get_motor_parameters())
        self._send_specific_command(self.get_motor_parameters())
        # slight delay to ensure connection!
        time.sleep(0.2)
        return self._updated_async

    def update_async(self, data):
        self._updated_async = True
        packet_ID = data[0]
        if (packet_ID == COMM_PACKET_ID.COMM_GET_VALUES) or (packet_ID == COMM_PACKET_ID.COMM_GET_VALUES_SETUP):
            self.parse_motor_parameters_async(data)
            # calculate acceleration
            now = time.time()
            dt = now - self._last_update_time
            self._motor_state_async.acceleration = self._motor_state_async.speed/dt
            self._last_update_time = now
        elif (packet_ID == COMM_PACKET_ID.COMM_ROTOR_POSITION):
            self.parse_position_feedback_async(data)

        if self._motor_state.error != 0:
            raise RuntimeError(ERROR_CODES[self._motor_state.error])
    
    def send_command(self):
        if not (self._command is None):
            # use the thread-safe method to write the command, so that we don't access the serial 
            # object while the reader thread is using it!!
            # TODO when pyserial adds full asyncio support, consider switching to that
            self._reader_thread.write(self._command)

    def _send_specific_command(self, command):
        if not (self._command is None):
            # use the thread-safe method to write the command, so that we don't access the serial 
            # object while the reader thread is using it!!
            # TODO when pyserial adds full asyncio support, consider switching to that
            self._reader_thread.write(command)

    def update(self):
        if not self._entered:
            raise RuntimeError("Tried to update motor state before safely powering on for device: " + self.device_info_string())
        
        # send the user specified command
        self.send_command()

        # send the command to get parameters (message will be read in other thread)
        self._send_specific_command(self.get_motor_parameters())

        # synchronize user-facing state with most recent async state
        # TODO implement some filtering on the async state, if it gets multiple updates
        # between user requested updates
        self._motor_state = self._motor_state_async

        # self.w.writerow([time.time()-self._start_time])
        
        
    # comm protocol commands
    def power_on(self):
        self._command = bytearray([0x40, 0x80, 0x20, 0x02, 0x21, 0xc0])
        self.send_command()

    def power_off(self):
        self.set_duty_cycle(0.0)
        self.send_command()

    def enter_idle_mode(self):
        self._command = None

    def enter_velocity_control(self):
        self._control_state = SERVO_SERIAL_CONTROL_STATE.VELOCITY

    def enter_position_control(self):
        self._control_state = SERVO_SERIAL_CONTROL_STATE.POSITION

    def enter_position_velocity_control(self):
        self._control_state = SERVO_SERIAL_CONTROL_STATE.POSITION_VELOCITY

    def enter_current_loop_control(self):
        self._control_state = SERVO_SERIAL_CONTROL_STATE.CURRENT_LOOP

    def enter_duty_cycle_control(self):
        self._control_state = SERVO_SERIAL_CONTROL_STATE.DUTY_CYCLE

    def set_duty_cycle(self, duty, set_command=True):
        buffer=[]
        buffer_append_int32(buffer, int(duty * 100000.0))
        data = [COMM_PACKET_ID.COMM_SET_DUTY] + buffer
        if set_command:
            self._command = bytearray(create_frame(data))
        return self._command

    def set_speed_ERPM(self, speed, set_command=True):
        buffer=[]
        buffer_append_int32(buffer, int(speed))
        data = [COMM_PACKET_ID.COMM_SET_RPM] + buffer
        if set_command:
            self._command = bytearray(create_frame(data))
        return self._command

    def set_current_loop(self, current, set_command=True):
        buffer=[]
        buffer_append_int32(buffer, int(current*1000.0))
        data = [COMM_PACKET_ID.COMM_SET_CURRENT] + buffer
        if set_command:
            self._command = bytearray(create_frame(data))
        return self._command

    def set_position(self, pos, set_command=True):
        buffer=[]
        buffer_append_int32(buffer, int(pos*1000000))
        data = [COMM_PACKET_ID.COMM_SET_POS] + buffer
        if set_command:
            self._command = bytearray(create_frame(data))
        return self._command

    def set_position_velocity(self, pos, vel, acc, set_command=True):
        # 4 byte pos * 1000, 4 byte vel * 1, 4 byte a * 1
        buffer=[]
        buffer_append_int32(buffer, int(pos*1000000))
        buffer_append_int32(buffer, int(vel))
        buffer_append_int32(buffer, int(acc))
        data = [COMM_PACKET_ID.COMM_SET_POS_SPD] + buffer
        if set_command:
            self._command = bytearray(create_frame(data))
        return self._command

    def set_multi_turn(self, set_command=True):
        cmd = bytearray([0x02 ,0x05 ,0x5C ,0x00 ,0x00 ,0x00 ,0x00 ,0x9E ,0x19 ,0x03])
        if set_command:
            self._command = cmd
        return cmd

    def set_zero_position(self, set_command=True):
        cmd = bytearray([0x02, 0x02, 0x5F, 0x01, 0x0E, 0xA0, 0x03])
        if set_command:
            self._command = bytearray([0x02, 0x02, 0x5F, 0x01, 0x0E, 0xA0, 0x03])
        return cmd
    
    def set_motor_parameter_return_format_all(self, set_command=True):
        header = COMM_PACKET_ID.COMM_GET_VALUES_SETUP
        data = [header, 0xFF,0xFF,0xFF,0xFF]
        cmd = bytearray(create_frame(data))
        if set_command:
            self._command = cmd
        return cmd

    def begin_position_feedback(self, set_command=True):
        if set_command:
            self._command = bytearray([0x02, 0x02, 0x0B, 0x04, 0x9C, 0x7E, 0x03])

    def get_motor_parameters(self, set_command=True):
        header = COMM_PACKET_ID.COMM_GET_VALUES
        data = [header]
        cmd = bytearray(create_frame(data))
        if set_command:
            self._command = cmd
        return cmd
        
    # comm data parsing 
    def parse_position_feedback_async(self, data):
        self._motor_state_async.position = -float(buffer_get_int32(data, 1))/1000.0

    def parse_motor_parameters_async(self, data):
        i = 1
        self._motor_state_async.mos_temperature = float(buffer_get_int16(data,i))/10.0
        i+=2
        self._motor_state_async.motor_temperature = float(buffer_get_int16(data,i))/10.0
        i+=2
        self._motor_state_async.output_current = float(buffer_get_int32(data,i))/100.0
        i+=4
        self._motor_state_async.input_current = float(buffer_get_int32(data,i))/100.0
        i+=4
        self._motor_state_async.id_current = float(buffer_get_int32(data,i))/100.0
        i+=4
        self._motor_state_async.iq_current = float(buffer_get_int32(data,i))/100.0
        i+=4
        self._motor_state_async.duty = float(buffer_get_int16(data,i))/1000.0
        i+=2
        self._motor_state_async.speed = float(buffer_get_int32(data,i))
        i+=4
        self._motor_state_async.input_voltage = float(buffer_get_int16(data,i))/10.0
        i+=2 + 24
        # TODO investigate what's in the 24 reserved bytes? Maybe it's interesting to record?
        self._motor_state_async.error = np.uint(data[i])
        i+=1
        self._motor_state_async.position_set = float(buffer_get_int32(data,i))/1000000.0
        i+=4
        self._motor_state_async.controlID = np.uint(data[i])
        i+=1 + 6
        self._motor_state_async.Vd = float(buffer_get_int32(data,i))/1000.0
        i+=4
        self._motor_state_async.Vq = float(buffer_get_int32(data,i))/1000.0
        i+=4

    # getters for motor state
    def get_temperature_celsius(self):
        """
        Returns:
        The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.mos_temperature
    
    def get_motor_error_code(self):
        """
        Returns:
        The most recently updated motor error code.
        Note the program should throw a runtime error before you get a chance to read
        this value if it is ever anything besides 0.

        Codes:
        
        """
        return self._motor_state.error

    def get_current_qaxis_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.iq_current

    def get_current_daxis_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.id_current

    def get_current_bus_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.input_current

    def get_voltage_qaxis_volts(self):
        """
        Returns:
        The most recently updated qaxis voltage in volts
        """
        return self._motor_state.Vq

    def get_voltage_daxis_volts(self):
        """
        Returns:
        The most recently updated daxis voltage in volts
        """
        return self._motor_state.Vd

    def get_voltage_bus_volts(self):
        """
        Returns:
        The most recently updated input voltage in volts
        """
        return self._motor_state.input_current

    def get_output_angle_radians(self):
        """
        Returns:
        The most recently updated output angle in radians
        """
        return self._motor_state.position * self.rad_per_Eang

    def get_output_velocity_radians_per_second(self):
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.speed * self.radps_per_ERPM

    def get_output_acceleration_radians_per_second_squared(self):
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration * self.radps_per_ERPM

    def get_output_torque_newton_meters(self):
        """
        Returns:
            the most recently updated output torque in Nm
        """
        return self.get_current_qaxis_amps()*self.motor_params["Kt"]*self.motor_params["GEAR_RATIO"]


    # user facing setters 
    def set_output_velocity_radians_per_second(self, vel):
        """
        Make motor go brrr

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(vel) >= self.motor_params["V_max"]:
            raise RuntimeError("Cannot control using speed mode for angles with magnitude greater than " + str(self.motor_params["V_max"]) + "rad/s!")

        if self._control_state not in [SERVO_SERIAL_CONTROL_STATE.VELOCITY]:
            raise RuntimeError("Attempted to send speed command without entering speed control " + self.device_info_string()) 

        self.set_speed_ERPM(vel/self.radps_per_ERPM)

    def set_duty_cycle_percent(self, duty):
        """
        Make motor go brrr

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(duty) >= 1:
            raise RuntimeError("Cannot control using duty cycle mode for more than 100 percent duty!")

        if self._control_state not in [SERVO_SERIAL_CONTROL_STATE.DUTY_CYCLE]:
            raise RuntimeError("Attempted to duty cycle command without entering duty cycle control " + self.device_info_string()) 

        self.set_duty_cycle(duty)

    def set_motor_current_qaxis_amps(self, curr):
        """
        Make motor go brrr

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(curr) >= self.motor_params["Curr_max"]:
            raise RuntimeError("Cannot control using current mode with magnitude greater than " + str(self.motor_params["I_max"]) + "rad/s!")

        if self._control_state not in [SERVO_SERIAL_CONTROL_STATE.CURRENT_LOOP]:
            raise RuntimeError("Attempted to send current command without entering current control " + self.device_info_string()) 

        self.set_current_loop(curr)

    def set_output_angle_radians(self, pos, vel=1000, acc=500):
        """
        Make motor go brrr

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(pos) >= self.motor_params["P_max"]:
            raise RuntimeError("Cannot control using position mode for angles with magnitude greater than " + str(self.motor_params["P_max"]) + "rad!")

        if self._control_state == SERVO_SERIAL_CONTROL_STATE.POSITION:
            self.set_position_velocity(pos, vel, acc)
        elif self._control_state == SERVO_SERIAL_CONTROL_STATE.POSITION_VELOCITY:
            self.set_position(pos)
        else:
            raise RuntimeError("Attempted to send position command without entering position control " + self.device_info_string()) 

    def set_output_torque_newton_meters(self, torque):
        """
        Used for either current or MIT Mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.
        
        Args:
            torque: The desired output torque in Nm.
        """
        self.set_motor_current_qaxis_amps( (torque/self.motor_params["Kt"]/self.motor_params["GEAR_RATIO"]) )

    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque):
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque
        
        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque_newton_meters(torque*self.motor_params["Kt"])

    def set_motor_angle_radians(self, pos):
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle
        
        Args:
            pos: The desired motor-side position in rad.
        """
        self.set_output_angle_radians(pos/(self.motor_params["GEAR_RATIO"]) )

    def set_motor_velocity_radians_per_second(self, vel):
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity
        
        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity_radians_per_second(vel/(self.motor_params["GEAR_RATIO"]) )

    def get_motor_angle_radians(self):
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle
        
        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position*self.motor_params["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self):
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity
        
        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity*self.motor_params["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self):
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration
        
        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration*self.motor_params["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self):
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque
        
        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.get_output_torque_newton_meters()*self.motor_params["GEAR_RATIO"]

    # Pretty stuff
    def __str__(self):
        """Prints the motor's device info and current"""
        return self.device_info_string() + " | Position: " + '{: 1f}'.format(round(self.get_output_angle_radians(),3)) + " rad | Velocity: " + '{: 1f}'.format(round(self.get_output_velocity_radians_per_second(),3)) + " rad/s | current: " + '{: 1f}'.format(round(self._motor_state.iq_current,3)) + " A | temp: " + '{: 1f}'.format(round(self._motor_state.mos_temperature,0)) + " C"

    def device_info_string(self):
        """Prints the motor's serial port and device type."""
        return f"{self.motor_params['Type']} Port: {self.port}"

    # controller variables
    T = property(get_temperature_celsius, doc="temperature_degrees_C")
    """Temperature in Degrees Celsius"""

    # TODO write actual codes in description here as well
    error = property(get_motor_error_code, doc="Error")
    """Motor error code. 0 means no error."""

    # electrical variables
    iq = property(get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps")
    """Q-axis current in amps"""

    id = property(get_current_daxis_amps, doc="current_daxis_amps")
    """D-axis current in amps"""

    ibus = property(get_current_bus_amps, doc="current_bus_amps")
    """Bus input current in amps"""

    vq = property(get_voltage_qaxis_volts, doc="voltage_qaxis_volts")
    """Q-axis voltage in volts"""

    vd = property(get_voltage_daxis_volts, doc="voltage_daxis_volts")
    """D-axis voltage in volts"""

    vbus = property(get_voltage_bus_volts, doc="voltage_bus_volts")
    """Bus input voltage in volts"""

    # output-side variables
    θ = property(get_output_angle_radians, set_output_angle_radians, doc="output_angle_radians")
    """Output angle in rad"""

    θd = property (get_output_velocity_radians_per_second, set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    """Output velocity in rad/s"""

    θdd = property(get_output_acceleration_radians_per_second_squared, doc="output_acceleration_radians_per_second_squared")
    """Output acceleration in rad/s/s"""

    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters, doc="output_torque_newton_meters")
    """Output torque in Nm"""

    # motor-side variables
    ϕ = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians")
    """Motor-side angle in rad"""
    
    ϕd = property (get_motor_velocity_radians_per_second, set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    """Motor-side velocity in rad/s"""

    ϕdd = property(get_motor_acceleration_radians_per_second_squared, doc="motor_acceleration_radians_per_second_squared")
    """Motor-side acceleration in rad/s/s"""

    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters, doc="motor_torque_newton_meters")
    """Motor-side torque in Nm"""


    














