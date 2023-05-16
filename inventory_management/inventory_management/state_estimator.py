#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
import math

class StateEstimatorNode(Node):
    def __init__(self,name, sub_accel_name, sub_gyro_name, sub_vel_name, pub_accel_name, pub_vel_name,pub_pos_name, \
                pub_ang_accel_name,pub_ang_vel_name,pub_angle_name):
        super().__init__(name)
        self._raw_accel_subscriber = self.create_subscription(Vector3,sub_accel_name,self._raw_accel_callback,10)
        self._raw_gyro_subscriber = self.create_subscription(Vector3,sub_gyro_name,self._raw_gyro_callback,10)
        self._raw_vel_subscriber = self.create_subscription(Float64,sub_vel_name,self._raw_vel_callback,10)
        self._kal_accel_publisher = self.create_publisher(Float64,pub_accel_name,10)
        self._kal_vel_publisher = self.create_publisher(Float64,pub_vel_name,10)
        self._kal_pos_publisher = self.create_publisher(Vector3,pub_pos_name,10)
        self._kal_ang_accel_publisher = self.create_publisher(Float64,pub_ang_accel_name,10)
        self._kal_ang_vel_publisher = self.create_publisher(Float64,pub_ang_vel_name,10)
        self._kal_angle_publisher = self.create_publisher(Float64,pub_angle_name,10)

        self.dt = 0.01
        self._setup_subscriber_variables()
        self._setup_kalman_filter()
        self._accel_kal = Float64()
        self._vel_kal = Float64()
        self._pos_kal = Vector3()
        self._ang_accel_kal = Float64()
        self._ang_vel_kal = Float64()
        self._angle_kal = Float64()

        self.get_logger().info("State Estimator Node started")
        self.timer = self.create_timer(self.dt,self.publish_state_estimate)

    def _raw_accel_callback(self,msg):
        self._raw_accel_x = round(msg.x,2)
        self._raw_accel_y = round(msg.y,2)
        self._raw_accel_z = round(msg.z,2)
        
    def _raw_gyro_callback(self,msg):
        self._raw_gyro_x = round(msg.x,2)
        self._raw_gyro_y = round(msg.y,2)
        self._raw_gyro_z = -round(msg.z,2)

    def _raw_vel_callback(self,msg):
        self._raw_vel = msg.data

    def publish_state_estimate(self):
        x_prediction = self._predict_states()
        p_prediction = self._predict_error_covariance()

        z = np.array([[self._raw_vel],
                     [self._raw_gyro_z]])
        
        K = self._calculate_kalman_gain(p_prediction)

        self._x_states = self._estimate_states(x_prediction,K,z)

        self._P = self._estimate_covariance(K,p_prediction)

        dist_kal = float(self._x_states[0])
        vel_kal = float(self._x_states[1])
        accel_kal= float(self._x_states[2])
        angle_kal = float(self._x_states[3])
        ang_vel_kal = float(self._x_states[4])
        ang_accel_kal = float(self._x_states[5])
        
        angle_kal_deg = self._convert_angle_rad_to_deg(angle_kal)
        wrapped_angle_kal_deg = self._wrap_angle_deg(angle_kal_deg)
        pos_x, pos_y = self._get_position(dist_kal,wrapped_angle_kal_deg)

        self._accel_kal.data = accel_kal
        self._vel_kal.data = vel_kal
        self._pos_kal.x = pos_x
        self._pos_kal.y = pos_y
        self._pos_kal.z = 0.0
        self._ang_accel_kal.data = ang_accel_kal
        self._ang_vel_kal.data = ang_vel_kal
        self._angle_kal.data = wrapped_angle_kal_deg

        self._kal_accel_publisher.publish(self._accel_kal)
        self._kal_vel_publisher.publish(self._vel_kal)
        self._kal_pos_publisher.publish(self._pos_kal)
        self._kal_ang_accel_publisher.publish(self._ang_accel_kal)
        self._kal_ang_vel_publisher.publish(self._ang_vel_kal)
        self._kal_angle_publisher.publish(self._angle_kal)

        self._prev_pos_x = pos_x
        self._prev_pos_y = pos_y
        
    def _setup_subscriber_variables(self):
        self._raw_accel_x = 0.0
        self._raw_accel_y = 0.0
        self._raw_accel_z = 0.0
        self._raw_gyro_x = 0.0
        self._raw_gyro_y = 0.0
        self._raw_gyro_z = 0.0
        self._raw_vel = 0.0
        self._prev_pos_x = 0.0
        self._prev_pos_y = 0.0

    def _setup_kalman_filter(self):
        """
        Initial state matrix (n_x X 1) involving:
        -distance
        -velocity
        -acceleration
        -yaw angle
        -yaw angular velocity
        -yaw angular acceleration
        """
        self._x_states = np.array([[0],[0],[0],[0],[0],[0]])
        
        self._number_of_states = 6
        self._number_of_measurements = 2

        """
        Definition of state transition matrix (n_x X n_x)
        """
        self._F = np.array([[0, self.dt, 0.5*pow(self.dt,2), 0, 0, 0],
                            [0, 1, self.dt, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, self.dt, 0.5*pow(self.dt,2)],
                            [0, 0, 0, 0, 1, self.dt],
                            [0, 0, 0, 0, 0, 1]])

        """
        Definition of covariance matrix 
        """
        self._P = np.eye(self._number_of_states)*500

        """
        Definition of process noise matrix
        """
        self._Q = np.eye(self._number_of_states)*0.01

        """
        Definition of measurement noise matrix
        """
        self._R = np.eye(self._number_of_measurements)*0.1

        """
        Definition of observation matrix
        """
        self._H = np.array([[0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]])

    def _convert_angle_rad_to_deg(self,angle):
        return math.degrees(angle)
    
    def _wrap_angle_deg(self,angle):
        multiplier = np.floor(angle/360)
        return angle - (multiplier*360)
    
    def _get_position(self,distance,angle):
        if angle <= 360 and angle > 270:
            angle_relative = math.radians(angle - 270)
            pos_x = -distance*np.cos(angle_relative) + self._prev_pos_x
            pos_y = distance*np.sin(angle_relative) + self._prev_pos_y
        elif angle <= 270 and angle > 180:
            angle_relative = math.radians(angle - 180)
            pos_x = -distance*np.sin(angle_relative) + self._prev_pos_x
            pos_y = -distance*np.cos(angle_relative) + self._prev_pos_y
        elif angle <= 180 and angle > 90:
            angle_relative = math.radians(angle - 90)
            pos_x = distance*np.cos(angle_relative) + self._prev_pos_x
            pos_y = -distance*np.sin(angle_relative) + self._prev_pos_y
        elif angle <= 90 and angle >= 0:
            angle_relative = math.radians(angle)
            pos_x = distance*np.sin(angle_relative) + self._prev_pos_x
            pos_y = distance*np.cos(angle_relative) + self._prev_pos_y
        return pos_x,pos_y
    def _predict_states(self):
        return np.linalg.multi_dot([self._F, self._x_states])

    def _predict_error_covariance(self):
        return np.linalg.multi_dot([self._F,self._P,np.transpose(self._F)]) + self._Q

    def _calculate_kalman_gain(self, p_pred):
        #Innovation covariance
        S = np.linalg.multi_dot([self._H,p_pred,np.transpose(self._H)]) + self._R
        return np.linalg.multi_dot([p_pred,np.transpose(self._H),np.linalg.inv(S)])

    def _estimate_states(self,x_pred,K,z):
        #Innvoation residual
        y = z-np.linalg.multi_dot([self._H,x_pred])
        return x_pred + np.linalg.multi_dot([K,y])

    def _estimate_covariance(self,K,p_pred):
        return np.linalg.multi_dot([np.identity(self._number_of_states)-np.linalg.multi_dot([K,self._H]),p_pred])

def main(args=None):
    rclpy.init(args=args)

    node = StateEstimatorNode("state_estimator_node","raw_accel","raw_gyro","raw_vel","kal_accel","kal_vel","kal_pos","kal_ang_accel","kal_ang_vel","kal_angle")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
