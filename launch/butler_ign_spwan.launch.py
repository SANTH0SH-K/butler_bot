#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    butler_path = get_package_share_directory("butler_bot")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    lidar_enabled = LaunchConfiguration("lidar_enabled", default=True)

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
           {'robot_description':Command( \
                    ['xacro ', join(butler_path, 'urdf/butler_bot.xacro'),
	   ])}],
        remappings=[
            ('/joint_states', '/joint_states'),
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
           {'robot_description':Command( \
                    ['xacro ', join(butler_path, 'urdf/butler_bot.xacro'), 
	   ]),
	   'use_sim_time' : use_sim_time
	   }],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "apmr",
            "-allow_renaming", "true",
            "-z", "0.2",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )
  # Bridge between Ignition and ROS 2
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/default/model/apmr/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/apmr/joint_state', '/joint_states'),
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/imu', '/imu'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])


    
    return LaunchDescription([
        DeclareLaunchArgument("lidar_enabled", default_value = lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="-0.4889"),
        DeclareLaunchArgument("position_y", default_value="-1.9888"),
        DeclareLaunchArgument("orientation_yaw", default_value="1.5972"),

        DeclareLaunchArgument("use_sim_time", default_value="true"),
        robot_state_publisher,
        joint_state_publisher_node,
        gz_spawn_entity, gz_ros2_bridge
    ])
