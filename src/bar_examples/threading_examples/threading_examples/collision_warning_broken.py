#!/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

import numpy as np


class CollisionWarning(Node):

    def __init__(self):
        super().__init__('collision_warning')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.close = False
        self.trigger_service = self.create_service(Trigger, '/test_collision', self.trigger_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.client_service = self.create_client(Trigger, '/test_collision')

    def lidar_callback(self, msg):
        self.get_logger().info(f'I heard: {msg}')

        min_range = np.min(msg.ranges)
        if min_range < 1:
            self.close = True
        else: 
            self.close = False


    def timer_callback(self): 
        self.client_service.call(Trigger.Request())

    def trigger_callback(self, req, res): 
        if self.close: 
            msg = Twist()
            self.vel_pub.publish(msg)
        return res

def main(args=None):
    rclpy.init(args=args)

    collision_warning = CollisionWarning()

    rclpy.spin(collision_warning)

    rclpy.shutdown()


if __name__ == '__main__':
    main()