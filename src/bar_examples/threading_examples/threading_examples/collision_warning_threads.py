#!/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
        self.trigger_service = self.create_service(Trigger, '/test_collision', self.trigger_callback, )
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.client_service = self.create_client(Trigger, '/test_collision', callback_group=MutuallyExclusiveCallbackGroup())

    def lidar_callback(self, msg):
        self.get_logger().info(f'Lidar msg received ')

        min_range = np.min(msg.ranges)
        if min_range < 1:
            self.get_logger().info(f'Too close')

            self.close = True
        else: 
            self.close = False
            self.get_logger().info(f'Far enough away')


    def timer_callback(self): 
        self.get_logger().info(f'Timer running')

        self.client_service.call(Trigger.Request())

    def trigger_callback(self, req, res): 
        self.get_logger().info(f'Service request running')
        if self.close: 
            msg = Twist()
            self.vel_pub.publish(msg)
            
        return res

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    collision_warning = CollisionWarning()
    executor.add_node(collision_warning)

    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()