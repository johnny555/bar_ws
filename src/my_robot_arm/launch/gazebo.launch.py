import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from os.path import join
from launch.actions import IncludeLaunchDescription, ExecuteProcess


def generate_launch_description():

    resources_package = 'my_robot_arm'

    # Make path to resources dir without last package_name fragment.
    path_to_share_dir_clipped = ''.join(get_package_share_directory(resources_package).rsplit('/' + resources_package, 1))


    # Gazebo Sim.
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(get_package_share_directory('my_robot_arm'), 'urdf', 'coke_pickup.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=dict(gz_args=f'-r {world_file} --verbose').items(),
        )

    # Spawn
    spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot_arm.urdf',
                '-x', '1.0',
                '-z', '1.0',
                '-Y', '2.4',
                '-topic', '/robot_description'
                ],
            output='screen',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    use_rviz = LaunchConfiguration('use_rviz')

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value='false')

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(resources_package),
                    'launch',
                    'description.launch.py'
                    ])
                ]),
            condition=UnlessCondition(use_rviz),  # rviz launch includes rsp.
            launch_arguments=dict(use_sim_time=use_sim_time).items(),
            )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(resources_package),
                'launch',
                'display.launch.py'
            ])
        ]),
        condition=IfCondition(use_rviz),
        )
    
        # Step 5: Enable the ros2 controllers
    start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=['joint_state_broadcaster', 'gripper_controller', 'arm_controller'],
                output="screen",
            )


    move_group = IncludeLaunchDescription(join(get_package_share_directory("my_robot_arm_moveit"), "launch", "move_group.launch.py"))
    rviz = IncludeLaunchDescription(join(get_package_share_directory("my_robot_arm_moveit"), "launch", "moveit_rviz.launch.py"))
    
    mg_sim_time = ExecuteProcess(cmd=["ros2", "param", "set", "/move_group", "use_sim_time","True"])

    rviz_sim_time = ExecuteProcess(cmd=["ros2", "param", "set", "/rviz", "use_sim_time","True"])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked', ],
        output='screen'        )
    
    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0",  "realsense_link", "my_robot_arm.urdf/realsense_link/realsense_d435"])

    return LaunchDescription([
        use_sim_time_launch_arg,
        use_rviz_arg,
        robot_state_publisher,
        rviz,
        gazebo,
        spawn,
        start_controllers,
        move_group,
        mg_sim_time,
        bridge,
        static_pub,
        #rviz_sim_time
    ])
