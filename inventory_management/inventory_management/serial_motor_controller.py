#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class SerialMotorControllerNode(Node):
    def __init__(self,name, sub_name):
        super().__init__(name)
        self._twist_subscriber = self.create_subscription(Twist,sub_name,self.twist_callback,10)
        # Initiate serial connection
        self._serial_conn = serial.Serial('/dev/ttyACM0',115200,timeout=1.0)
        # Wait 3 seconds for Arduino to restart
        time.sleep(3)
        # Reset input buffer
        self._serial_conn.reset_input_buffer()   
        self.get_logger().info("Serial Connection Established")
        self.get_logger().info("Serial Motor Controller Node started")

    def __del__(self):
        # Close serial connection
        self._serial_conn.close()
        print("Serial Connection Closed")

    def twist_callback(self,msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info("Linear_x: {} | Angular_z: {}".format(linear_x,angular_z))
        self._serial_conn.write("{},{}\n".format(linear_x,angular_z).encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)

    node = SerialMotorControllerNode("serial_motor_controller_node","twist")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
