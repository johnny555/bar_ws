#!/bin/python3
import rclpy
from rclpy.node import Node


from sensor_msgs.msg import LaserScan

class BasicSubscriber(Node):

    def __init__(self):
        super().__init__('basic_subscriber')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.listener_callback,
            10)
        

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg}')

def main(args=None):
    rclpy.init(args=args)

    basic_subscriber = BasicSubscriber()

    rclpy.spin(basic_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()