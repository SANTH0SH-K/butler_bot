#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    butler_path = get_package_share_directory("butler_bot")
    world_file = LaunchConfiguration("world_file", default = join(butler_path, "worlds", "cafeworld.sdf"))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    spawn_butler_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(butler_path, "launch", "butler_ign_spwan.launch.py")),
        launch_arguments={
            # Pass any arguments if your spawn.launch.py requires
        }.items()
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=join(butler_path, "worlds")),

        AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=join(butler_path, "models")),

        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        
         gz_sim, spawn_butler_node
    ])
