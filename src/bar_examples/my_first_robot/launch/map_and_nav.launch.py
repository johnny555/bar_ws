from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnExecutionComplete

from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


from os.path import join


def generate_launch_description():
    # This allows us to have the rviz_config as an argument on the command line
    rviz_config_arg =  DeclareLaunchArgument(
        'rviz_config', default_value="nav.yaml"
    )
    # This allows us to use the rviz_config variable in substitutions in this launch description.
    rviz_config = LaunchConfiguration('rviz_config', default="view.rviz")

    base_path = get_package_share_directory("my_first_robot")
    # Include the gazebo setup
    gazebo = IncludeLaunchDescription(join(base_path, "launch","gazebo.launch.py"))

    # SLAM Toolbox for mapping
    slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource( 
                    join(get_package_share_directory('slam_toolbox'), 
                          'launch','online_async_launch.py')),
            launch_arguments=[
                    ('slam_params_file', join(get_package_share_directory('my_first_robot'),'config','mapping.yaml'))
                         ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([base_path, 'config', 'nav.rviz'])
        ]
    )


    
    # Nav2 bringup for navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(get_package_share_directory('nav2_bringup'), 'launch','navigation_launch.py')),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'use_sim_time': 'true',
            'params_file': get_package_share_directory('my_first_robot') + '/config/navigation.yaml',
            'autostart': 'True'
        }.items(),
    )
    #remap = SetRemap("cmd_vel","/diff_drive_controller/cmd_vel")

    nav_remapped = GroupAction(actions=[#remap, 
        navigation])

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper.py",
        remappings=[("/cmd_vel_in", "/cmd_vel_nav"),
                       ("/cmd_vel_out",  "/diff_drive_base_controller/cmd_vel")],
        parameters=[{"use_sim_time","True"}],
        output="screen",
        name="twist_stamper_nav"
    )  



    return LaunchDescription([gazebo, slam_toolbox, 
                                rviz, rviz_config_arg, nav_remapped, twist_stamper])