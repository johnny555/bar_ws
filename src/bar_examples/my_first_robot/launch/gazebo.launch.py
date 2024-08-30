import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_param_builder import load_xacro

from launch_ros.actions import Node

from pathlib import Path
from os.path import join

def generate_launch_description():

    resources_package = 'forklift_freecad'

    # Make path to resources dir without last package_name fragment.
    path_to_share_dir_clipped = ''.join(get_package_share_directory(resources_package).rsplit('/' + resources_package, 1))

    # Gazebo hint for resources.
    os.environ['GZ_SIM_RESOURCE_PATH'] = path_to_share_dir_clipped

    # Ensure `SDF_PATH` is populated since `sdformat_urdf` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    cafe_world_uri = join(get_package_share_directory("my_first_robot"), "worlds", "warehouse.sdf")
   

    # Gazebo Sim.
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=dict(gz_args=f"-r {cafe_world_uri} --verbose").items(),
        )

    # Spawn
    spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'FirstRobot',
                '-z', '0.5',
                '-topic', '/robot_description'
                ],
            output='screen',
    )

    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    
    rqt_robot_steering = Node(package="rqt_robot_steering",
                              executable="rqt_robot_steering")


    robot_file = join(get_package_share_directory("my_first_robot"), "robot_description","first_robot.urdf.xacro")
    robot_xml = load_xacro(Path(robot_file))


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_xml
            },
            ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/FirstRobot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',],
        output='screen',
        remappings=[('/model/FirstRobot/cmd_vel','/cmd_vel')]
    )
    
    # Step 5: Enable the ros2 controllers
    start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=['joint_state_broadcaster', 'diff_drive_base_controller', 'tines_controller'],
                output="screen",
            )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper.py",
        remappings=[("/cmd_vel_in", "/cmd_vel"),
                       ("/cmd_vel_out",  "/diff_drive_base_controller/cmd_vel")],
        parameters=[{"use_sim_time","True"}],
        output="screen"
    )  


    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", "lidar_2d_link", "FirstRobot/base_footprint/lidar_2d_v1", ])
    
    static_pub2 = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0",  "realsense_link", "FirstRobot/base_footprint/realsense_d435"])

 


    return LaunchDescription([
        use_sim_time_launch_arg,
        robot_state_publisher,
        gazebo,
        spawn,
        rqt_robot_steering,
        bridge,
        start_controllers,
        twist_stamper,
        static_pub,
        static_pub2
    ])
