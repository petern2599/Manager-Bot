#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class SerialPublisherNode(Node):
    def __init__(self,name,pub_name_accel,pub_name_gyro, pub_name_vel):
        super().__init__(name)
        # Initiate serial connection
        self._serial_conn = serial.Serial('/dev/ttyACM0',115200,timeout=1.0)
        # Wait 3 seconds for Arduino to restart
        time.sleep(3)
        # Reset input buffer
        self._serial_conn.reset_input_buffer()   
        self.get_logger().info("Serial Connection Established")

        self.declare_parameter("ax_offset")
        self._ax_offset = self.get_parameter("ax_offset").value
        self.declare_parameter("ay_offset")
        self._ay_offset = self.get_parameter("ay_offset").value
        self.declare_parameter("az_offset")
        self._az_offset = self.get_parameter("az_offset").value
        self.declare_parameter("gx_offset")
        self._gx_offset = self.get_parameter("gx_offset").value
        self.declare_parameter("gy_offset")
        self._gy_offset = self.get_parameter("gy_offset").value
        self.declare_parameter("gz_offset")
        self._gz_offset = self.get_parameter("gz_offset").value

        self._raw_accel_publisher = self.create_publisher(Vector3,pub_name_accel,10)
        self._raw_gyro_publisher = self.create_publisher(Vector3,pub_name_gyro,10)
        self._raw_vel_publisher = self.create_publisher(Float64,pub_name_vel,10)
        self.get_logger().info("Serial Publisher Node started")

        self.raw_accel = Vector3()
        self.raw_gyro = Vector3()
        self.raw_vel = Float64()

        log_once = False

        while True:
            while self._serial_conn.in_waiting <= 0:
                time.sleep(0.01)
            imu_response = self._serial_conn.readline().decode('utf-8').rstrip()
            response_array = imu_response.split(",")
            if log_once == False:
                self.get_logger().info("Publishing data")
                log_once = True

            #Uncomment to check serial message received
            # print(imu_response)
            # print(response_array)

            self.raw_accel.x = float(response_array[0])-self._ax_offset
            self.raw_accel.y = float(response_array[1])-self._ay_offset
            self.raw_accel.z = float(response_array[2])-self._az_offset
            self.raw_gyro.x = float(response_array[3])-self._gx_offset
            self.raw_gyro.y = float(response_array[4])-self._gy_offset
            self.raw_gyro.z = float(response_array[5])-self._gz_offset
            self.raw_vel.data = (float(response_array[7])+float(response_array[8]))/2

            self._raw_accel_publisher.publish(self.raw_accel)
            self._raw_gyro_publisher.publish(self.raw_gyro)
            self._raw_vel_publisher.publish(self.raw_vel)

    def __del__(self):
        # Close serial connection
        self._serial_conn.close()
        print("Serial Connection Closed")

def main(args=None):
    rclpy.init(args=args)

    node = SerialPublisherNode("serial_imu_node","raw_accel","raw_gyro","raw_vel")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
