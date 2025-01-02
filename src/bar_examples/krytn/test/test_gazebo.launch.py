import unittest
import launch_testing
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import launch_testing.actions
import launch_testing.markers

@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_name = 'krytn'
    launch_file = os.path.join(
        get_package_share_directory(pkg_name),
        'launch',
        'gazebo.launch.py'
    )

    gazebo_launch = ExecuteProcess(
        cmd=['ros2', 'launch', launch_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        launch_testing.actions.ReadyToTest()
    ])

class TestGazeboLaunch(unittest.TestCase):
    def test_launch_file(self, proc_output):
        proc_output.assertWaitFor(
            'Configured and activated', 
            timeout=60,
            stream='stdout'
        )