import os
from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    jupyter = ExecuteProcess(
        cmd="jupyter lab --LabApp.token='' --notebook-dir=/workspace/src/jupyter-scaffold/notebooks/".split(' '),
        output="screen"
    )

    gazebo = IncludeLaunchDescription(join(get_package_share_directory('my_robot_arm'), 'launch','gazebo.launch.py'))


    return LaunchDescription([jupyter, gazebo])
