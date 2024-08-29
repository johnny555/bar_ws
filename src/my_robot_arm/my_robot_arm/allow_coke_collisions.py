#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2


def main():
    rclpy.init()
    node = rclpy.create_node(node_name="my_robot_arm_control")
    logger = node.get_logger()
    
    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(node=node, 
                      joint_names=['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint',
                                   'ur5_elbow_joint','ur5_wrist_1_joint',
                                   'ur5_wrist_2_joint','ur5_wrist_3_joint'],
                      base_link_name='ur5_base_link',
                      end_effector_name='gripper',
                      group_name='arm',
                      callback_group=callback_group )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(3.0).sleep()

    logger.info("Create collision box and allow collisions")

    obj_id = 'coke2'
    pos = [0.28, -0.5, 0.7]
    quart_xyzw = [0,0,0,1]

    moveit2.add_collision_cylinder (obj_id, height=0.12, radius=0.03, position=pos, quat_xyzw=quart_xyzw)
    moveit2.allow_collisions(obj_id, True)

    node.create_rate(5.0).sleep()

    logger.info(f"created object {obj_id} and allowed collisions")
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()