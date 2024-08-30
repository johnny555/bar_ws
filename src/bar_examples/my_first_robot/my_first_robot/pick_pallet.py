#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import time

from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from argparse import ArgumentParser


def create_goal(pos):

    traj = JointTrajectory()
    traj.joint_names = ["tines_joint"]

    traj.points = [JointTrajectoryPoint(positions=[pos])]
    
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    goal.goal_time_tolerance = Duration(sec=1)
    
    tol = [JointTolerance(name=n, position=0.001, velocity=0.01) for n in traj.joint_names]
    ptol = [JointTolerance(name=n, position=0.5, velocity=0.1) for n in traj.joint_names]

    goal.path_tolerance = ptol
    goal.goal_tolerance = tol
    
    return goal

pick_pos = PoseStamped()
pick_pos.pose.position.x = 1.5216999998129777
pick_pos.pose.position.y = 0.0

place_pos = PoseStamped()
place_pos.pose.position.x = 1.0
place_pos.pose.position.y = -3.16
#place_pos.pose.orientation.z = -0.7
#place_pos.pose.orientation.w = 1


rclpy.init()
node = Node("move_robot")
ac = ActionClient(node, FollowJointTrajectory, "/tines_controller/follow_joint_trajectory")

pub = node.create_publisher(PoseStamped, '/goal_pose', qos_profile=1)
clock = node.get_clock()
logger = node.get_logger()
time.sleep(1.0)    

logger.info("Waiting for action to be available")
ac.wait_for_server()

logger.info("Move Tines Down")
# Move tines down
goal = create_goal(-0.05)
res = ac.send_goal_async(goal)
rclpy.spin_until_future_complete(node, res)

logger.info("Move to pickup")
pick_pos.header.frame_id ='map'
pick_pos.header.stamp = clock.now().to_msg()
pub.publish(pick_pos)


time.sleep(5.0)    
logger.info("Move tines up")
# move tines up
goal = create_goal(0.1)
res = ac.send_goal_async(goal)
rclpy.spin_until_future_complete(node, res)

logger.info("Move to place position")
# Move to place pos
place_pos.header.frame_id ='map'
place_pos.header.stamp = clock.now().to_msg()
pub.publish(place_pos)

time.sleep(10.0)    

logger.info("Placeing package")
# move tines up
goal = create_goal(-0.05)
res = ac.send_goal_async(goal)
rclpy.spin_until_future_complete(node, res)

logger.info("Move to start position")
pub.publish(pick_pos)
