#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch_ros.actions import Node

def generate_launch_description():
    mm_map = '/home/katana/Desktop/array/mola_ws/map/FIBO.mm'
    simple_map = '/home/katana/Desktop/array/mola_ws/map/FIBO.simplemap'

    localize_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', "mola_lidar_odometry" ,
             'ros2-lidar-odometry-localize-katana.launch.py',
             'lidar_topic_name:=/livox/lidar_filtered',
             'imu_topic_name:=/imu',
             'use_rviz:=true',
             'use_mola_gui:=true',
             'enable_mapping:=false',
             'start_active:=false',
             f"mola_initial_map_mm_file:={mm_map}",
             f"mola_initial_map_sm_file:={simple_map}"],        
        output='screen'
    )
    
    plot_node = Node(
        package='mola_bringup',
        executable='plot_lidar_trajectory.py',
        name='plot_lidar_trajectory',
        output='screen'
    )
    filterpass = Node(
        package='mola_bringup',
        executable='filterpass.py',
        name='filterpass',
        output='screen'
    )


    
    ld = LaunchDescription()
    ld.add_action(filterpass)

    ld.add_action(localize_cmd)
    ld.add_action(plot_node)


    return ld

# def main(args=None):
#    generate_launch_description()

# if __name__ == '__main__':
#    main()
