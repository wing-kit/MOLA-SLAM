#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    filterpass = Node(
        package='mola_bringup',
        executable='filterpass.py',
        name='filterpass',
        output='screen'
    )

    # pass_through_filter_node = Node(
    #     package='mola_bringup',
    #     executable='intensity_passthrough_filter.py',
    #     name='intensity_passthrough_filter',
    #     output='screen'
    # )

    slam_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', "mola_lidar_odometry" ,
             'ros2-lidar-odometry-katana.launch.py',
             'lidar_topic_name:=/livox/lidar_filtered',
             'imu_topic_name:=/imu',
             'use_rviz:=true',
             'use_mola_gui:=true',
             'start_mapping_enabled:=true',
             'start_active:=true'],        
        output='screen'
    )
    
    plot_node = Node(
        package='mola_bringup',
        executable='plot_lidar_trajectory.py',
        name='plot_lidar_trajectory',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(filterpass)
    # ld.add_action(pass_through_filter_node)
    ld.add_action(slam_cmd)
    ld.add_action(plot_node)

    return ld
# def main(args=None):
#    generate_launch_description()

# if __name__ == '__main__':
#    main()