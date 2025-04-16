import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get directories
    butler_path = get_package_share_directory('butler_bot')
    nav2_path = get_package_share_directory('nav2_bringup')
    
    # Map configuration
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            butler_path,
            'maps',
            'cafe.yaml'))  # Replace with your actual map file
    
    # Parameters file
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            butler_path,
            'params',
            'butler.yaml'))  # Replace with your params file
    
    nav2_launch_file_dir = os.path.join(nav2_path, 'launch')

    # RViz configuration
    rviz_config_dir = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(
            butler_path,
            'config',
            'navigation.rviz'))  # Replace with your RViz config file

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Ignition) clock if true'),
            
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_dir,
            description='Full path to RViz config file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])