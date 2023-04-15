#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Vector3

class SerialIMUNode(Node):
    def __init__(self,name,pub_name_accel,pub_name_gyro):
        super().__init__(name)
        # Initiate serial connection
        self._serial_conn = serial.Serial('/dev/ttyACM0',115200,timeout=1.0)
        # Wait 3 seconds for Arduino to restart
        time.sleep(3)
        # Reset input buffer
        self._serial_conn.reset_input_buffer()   
        self.get_logger().info("Serial Connection Established")

        self._raw_accel_publisher = self.create_publisher(Vector3,pub_name_accel,10)
        self._raw_gyro_publisher = self.create_publisher(Vector3,pub_name_gyro,10)
        self.get_logger().info("Serial IMU Node started")

        self.raw_accel = Vector3()
        self.raw_gyro = Vector3()

        while True:
            while self._serial_conn.in_waiting <= 0:
                time.sleep(0.1)
            imu_response = self._serial_conn.readline().decode('utf-8').rstrip()
            response_array = imu_response.split(",")

            #Uncomment to check serial message received
            # print(imu_response)
            # print(response_array)

            self.raw_accel.x = float(response_array[0])
            self.raw_accel.y = float(response_array[1])
            self.raw_accel.z = float(response_array[2])
            self.raw_gyro.x = float(response_array[3])
            self.raw_gyro.y = float(response_array[4])
            self.raw_gyro.z = float(response_array[5])

            self._raw_accel_publisher.publish(self.raw_accel)
            self._raw_gyro_publisher.publish(self.raw_gyro)

    def __del__(self):
        # Close serial connection
        self._serial_conn.close()
        print("Serial Connection Closed")

def main(args=None):
    rclpy.init(args=args)

    node = SerialIMUNode("serial_imu_node","raw_accel","raw_gyro")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
