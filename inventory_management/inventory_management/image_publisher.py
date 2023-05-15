#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from threading import Thread

class CameraStream():
    def __init__(self,src=0):
        self.capture = cv2.VideoCapture('/dev/video{}'.format(src), cv2.CAP_V4L)
        if self.capture.isOpened() == False:
            print("Error in accessing camera")
            exit(0)
        self.capture.grab()
        self.ret, self.frame = self.capture.retrieve()
        if self.ret == False:
            print("No more frames to read")
            exit(0)
        self.stopped = False

    def start(self):
        self.thread = Thread(target = self.update, args=())
        self.thread.start()

    def update(self):
        while True:
            if self.stopped == True:
                print("Stopping")
                break
            else:
                self.capture.grab()
                self.ret, self.frame = self.capture.retrieve()
                
        
    def read(self):
        return self.ret,self.frame
    
    def get_dimension(self):
        width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width,height
    
    def set_size(self,width,height,percent):
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,int(width*(percent/100)))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,int(height*(percent/100)))
        
    def stop(self):
        self.stopped = True
    
    def release_camera(self):
        self.capture.release()

class ImagePublisherNode(Node):
    def __init__(self,name, pub_name):
        super().__init__(name)
        self.declare_parameter("camera_index")
        self.camera_index = self.get_parameter("camera_index").value
        self.declare_parameter("resize_percent")
        self.resize_percent = self.get_parameter("resize_percent").value

        self.image_publisher = self.create_publisher(Image,pub_name,10)
        self.get_logger().info("Node started")

        self.capture = CameraStream(self.camera_index)
        width, height = self.capture.get_dimension()
        self.capture.set_size(width,height,self.resize_percent)
        self.capture.start()
        print("Capture {} started".format(self.camera_index))

        self.bridge = CvBridge()
        self.counter = 0
        self.timer = self.create_timer(0.1,self.publish_image)
    
    def publish_image(self):
        ret, frame = self.capture.read()
        self.counter += 1
        self.get_logger().info("Publishing Frame {}".format(self.counter))
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame))

    def __del__(self):
        self.capture.stop()
        self.capture.release_camera()

def main(args=None):
    rclpy.init(args=args)

    node = ImagePublisherNode("image_publisher_node","video_frame")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
