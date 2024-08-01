#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile

import math
import geometry_msgs.msg
import tf2_ros
from sensor_msgs.msg import Temperature, Imu

from geometry_msgs.msg import TransformStamped

class Imu_to_tf(Node):
    def __init__(self):
        super().__init__("tf_broadcaster_imu")
        
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "world"
        self.transform_stamped.child_frame_id = "base_link"

        self.subscriber = self.create_subscription(Imu,'/imu/data_raw',self.handle_imu_pose,QoSProfile(depth=1,
                                                                                                   durability=DurabilityPolicy.VOLATILE,
                                                                                                   reliability=ReliabilityPolicy.BEST_EFFORT))
        self.br = tf2_ros.TransformBroadcaster(self)

    def handle_imu_pose(self,msg):

        q0 = msg.orientation.x
        q1 = msg.orientation.y
        q2 = msg.orientation.z
        q3 = msg.orientation.w

        self.transform_stamped.transform.rotation.x = q0
        self.transform_stamped.transform.rotation.y = q1
        self.transform_stamped.transform.rotation.z = q2
        self.transform_stamped.transform.rotation.w = q3

        self.br.sendTransform(self.transform_stamped)


if __name__ =='__main__':
    print("Se inicio el nodo de publicacion TF_imu...")
    rclpy.init()
    nodo_tf = Imu_to_tf()

    rclpy.spin(nodo_tf)
    rclpy.shutdown()