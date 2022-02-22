import can
import time
from enum import Enum





class CAN_Manager(object):
    
    debug = True
    
    MIT_Params = {
        'AK80-9':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -25.64,
            'V_max' : 25.64,
            'I_min' : -18.0,
            'I_max' : 18.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0
        }
    }

    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ will be called twice
    _instance = None
    def __new__(cls):
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            
        return cls._instance

    def __init__(self):
        # unsure if this will work
        print("INIT")
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        self.notifier = can.Notifier(bus=self.bus, listeners=[self.handle_message])
        self.subscribed_motors = {}
        pass
    
    def add_motor(self, motor):
        self.subscribed_motors[motor.ID] = motor

    # Utility methods for data manipulation
    @staticmethod
    def limit_value(value, min, max):
        if value > max:
            return max
        elif value < min:
            return min
        else:
            return value

    @staticmethod
    def float_to_uint(x,x_min,x_max,num_bits):
        x = CAN_Manager.limit_value(x,x_min,x_max)
        span = x_max-x_min
        # (x - x_min)*(2^num_bits)/span
        return int((x- x_min)*( float((1<<num_bits)/span)) )

    @staticmethod
    def uint_to_float(self, x,x_min,x_max,num_bits):
        span = x_max-x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x*span/((1<<num_bits)-1) + x_min)

    # CAN Sending Functionality
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

    def power_on(self, motor_id):
        self.send_MIT_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC])
        
    def power_off(self, motor_id):
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD])

    def zero(self, motor_id):
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    def MIT_controller(self, motor_id, position, velocity, Kp, Kd, I, motor_type):
        position_uint16 = CAN_Manager.float_to_uint(position, self.MIT_Params[motor_type]['P_min'], 
                                                    self.MIT_Params[motor_type]['P_max'], 16)
        velocity_uint12 = CAN_Manager.float_to_uint(velocity, self.MIT_Params[motor_type]['V_min'], 
                                                    self.MIT_Params[motor_type]['V_max'], 12)
        Kp_uint12 = CAN_Manager.float_to_uint(Kp, self.MIT_Params[motor_type]['Kp_min'], 
                                                    self.MIT_Params[motor_type]['Kp_max'], 12)
        Kd_uint12 = CAN_Manager.float_to_uint(Kd, self.MIT_Params[motor_type]['Kd_min'], 
                                                    self.MIT_Params[motor_type]['Kd_max'], 12)
        I_uint12 = CAN_Manager.float_to_uint(I, self.MIT_Params[motor_type]['I_min'], 
                                                    self.MIT_Params[motor_type]['I_max'], 12)

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            (velocity_uint12&0x00F<<4) | (Kp_uint12) >> 8,
            (Kp_uint12&0x0FF),
            (Kd_uint12) >> 4,
            (Kd_uint12&0x00F<<4) | (I_uint12) >> 8,
            (I_uint12&0x0FF)
        ]

        self.send_MIT_message(self, motor_id, data)
        
    
    def parse_MIT_message(self, data, motor_type):
        
        temp = None
        error = None
        position_uint = data[1] <<8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4]>>4) <<4 ) >> 4
        current_uint = data[4]&0x0F<<8 | data[5]

        if len(data)  == 8:
            temp = int(data[6])
            error = int(data[7])
        
        else:
            print("Not an MIT Mode State Message")

        position = CAN_Manager.uint_to_float(position_uint, self.MIT_Params[motor_type]['P_min'], 
                                            self.MIT_Params[motor_type]['P_max'], 16)
        velocity = CAN_Manager.uint_to_float(velocity_uint, self.MIT_Params[motor_type]['V_min'], 
                                            self.MIT_Params[motor_type]['V_max'], 12)
        current = CAN_Manager.uint_to_float(current_uint, self.MIT_Params[motor_type]['I_min'], 
                                            self.MIT_Params[motor_type]['I_max'], 12)

        if self.debug:
            print('  Position: ' + str(position))
            print('  Velocity: ' + str(velocity))
            print('  Current: ' + str(current))
            if (temp is not None) and (error is not None):
                print('  Temp: ' + str(temp))
                print('  Error: ' + str(error))

        return (position, velocity, current, temp, error)

    def handle_message(self, msg):
        data = bytes(msg.data)
        ID = data[0]
        if ID in self.subscribed_motors:
            self.subscribed_motors.update_state(self.parse_MIT_message(data, self.subscribed_motors[ID].type))




class TMotorManager():
    def __init__(self, motor_type='AK80-9', motor_ID=1, CSV_file=None):
        self.type = motor_type
        self.ID = motor_ID

        self.position = None
        self.velocity = None
        self.current = None
        self.temp = None
        self.error = None
        
        self.canman = CAN_Manager()
        self.canman.add_motor(self)


    def update_state(self, position, velocity, current, temp, error):
        # eventually hide these and account for units!
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temp = temp
        self.error = error

    
    def position_control(self, Kp, Kd, Pdes):
        self.canman.MIT_controller(self.ID,Pdes,0.0,Kp,Kd,0.0,self.type)

    def power_on(self):
        self.canman.power_on(self.ID)
    

       
if __name__ == "__main__":
    
    motor3 = TMotorManager(motor_type='AK80-9',motor_ID=3)

    motor3.power_on()
    # motor3.position_control(7.0, 0.8, 3.14/2)

    while(True):
        print(motor3.position)
        print(motor3.velocity)
        print(motor3.current)
        motor3.power_on()
        motor3.update_state()
        
        


        






