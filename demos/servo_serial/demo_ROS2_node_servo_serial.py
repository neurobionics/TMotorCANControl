# Example ROS2 Node that can switch between control modes via ros topics 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

from TMotorCANControl.servo_serial import *

CONTROL_MODES = {
    "POSITION": 0,
    "VELOCITY": 1,
    "CURRENT":  2,
    "DUTY":     3,
}

class MotorNode(Node):
    def __init__(self, device):
        super().__init__('motor_node')

        self.device = device
        self.dev_name = "dev"
        self.mode = 10   
        self.motor_data = 0

        self.device.set_zero_position()
        self.device.update()

        self.velocity_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/velocity",
            self.velocity_cb,
            10
        )

        self.duty_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/duty",
            self.duty_cb,
            10
        )

        self.position_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/position",
            self.position_cb,
            10
        )

        self.current_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/current",
            self.current_cb,
            10
        )

        self.output_pub = self.create_publisher(
            Float32MultiArray,
            f"{self.dev_name}/output",
            10
        )
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.mode == CONTROL_MODES["VELOCITY"]:
            self.device.set_output_velocity_radians_per_second(self.motor_data)
            self.device.update()
        elif self.mode == CONTROL_MODES["DUTY"]:
            self.device.set_duty_cycle_percent(self.motor_data)
            self.device.update()
        elif self.mode == CONTROL_MODES["POSITION"]:
            self.device.set_output_angle_radians(self.motor_data)
            self.device.update()
        elif self.mode == CONTROL_MODES["CURRENT"]:
            self.device.current_qaxis = self.motor_data
            self.device.update()

        output_angle = self.device.get_output_angle_radians()
        output_velocity = self.device.get_output_velocity_radians_per_second()
        output_acceleration = self.device.get_output_acceleration_radians_per_second_squared()
        output_torque = self.device.get_output_torque_newton_meters()
        
        output_msg = Float32MultiArray()
        output_msg.data = [output_angle, output_velocity, output_acceleration, output_torque]
        self.output_pub.publish(output_msg)

    def velocity_cb(self, msg):
        
        if self.mode != CONTROL_MODES["VELOCITY"]:
            self.mode = CONTROL_MODES["VELOCITY"]
            self.device.enter_velocity_control()
            self.device.update()
            print("vel")
            
        self.motor_data = msg.data

    def duty_cb(self, msg):
        
        if self.mode != CONTROL_MODES["DUTY"]:
            self.mode = CONTROL_MODES["DUTY"]
            self.device.enter_duty_cycle_control()
            self.device.update()
            print("duty")
            print(msg.data)
            
        self.motor_data = msg.data

    def position_cb(self, msg):
        
        if self.mode != CONTROL_MODES["POSITION"]:
            self.mode = CONTROL_MODES["POSITION"]
            self.device.enter_position_control()
            self.device.update()
            print("position")
            
        self.motor_data = msg.data

    def current_cb(self, msg):
        
        if self.mode != CONTROL_MODES["CURRENT"]:
            self.mode = CONTROL_MODES["CURRENT"]
            self.device.enter_current_control()
            self.device.update()
            print("current")
            
        self.motor_data = msg.data



def main(args=None):
    rclpy.init(args=args)

    with TMotorManager_servo_serial(port = "/dev/ttyUSB0") as dev:
        motor_node = MotorNode(device = dev)
        rclpy.spin(motor_node)

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
