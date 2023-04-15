#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisherNode(Node):
    def __init__(self,name, pub_name):
        super().__init__(name)
        self.image_publisher = self.create_publisher(Image,pub_name,10)
        self.timer = self.create_timer(0.1,self.publish_image)
        self.get_logger().info("Node started")
        self.capture = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def publish_image(self):
        ret, frame = self.capture.read()
        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.image_publisher.publish(self.br.cv2_to_imgmsg(frame))


def main(args=None):
    rclpy.init(args=args)

    node = ImagePublisherNode("image_publisher_node","video_frame")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
