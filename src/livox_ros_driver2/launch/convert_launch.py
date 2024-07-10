import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    
    pointcloud2laserscan = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [scanner, '/scan']),
            #             ('scan', [target_ns, '/scan'])],
            remappings=[('cloud_in', "/CloudMap"),
                        ('scan', "/scan")],

            parameters=[{
                'target_frame': 'laser',
                'transform_tolerance': 0.01,
                'min_height': 1.0,
                'max_height': 2.0,
                'angle_min': -1.44,  # less than -M_PI/2
                'angle_max': 1.436,  # less than M_PI/2
                'angle_increment': 0.008,  #  2.88 /360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    return LaunchDescription([

        pointcloud2laserscan,

    ])