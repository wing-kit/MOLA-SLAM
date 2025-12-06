#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_server = Node(
        package='mrpt_map_server',
        executable='map_server_node',
        parameters=[{
            'mm_file': '/home/carver/map/FIBO.mm',
            'frame_id': 'map',
            'pub_mm_only': True,
        }]
    )
    
    pf_loc = Node(
        package='mrpt_pf_localization',
        executable='mrpt_pf_localization_node',
        parameters=[{
            'global_frame_id': 'map',
            'odom_frame_id': 'odom', 
            'base_frame_id': 'base_link',
            'particles_count': 1000,
            'use_se3_pf': False,  # Missing parameter
        }],
        remappings=[
            ('/map', '/map'),
            ('/scan', '/livox/lidar'),
        ]
    )
    
    return LaunchDescription([map_server, pf_loc])