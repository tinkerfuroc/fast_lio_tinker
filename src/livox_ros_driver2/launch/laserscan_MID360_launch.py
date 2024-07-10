# This launch file is used to launch MID360 and convert pointcloud2 to laserscan
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'laser'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    # scanner = LaunchConfiguration("scanner") 
    # target_ns = LaunchConfiguration("target_ns")

    # declare_ns_cmd = DeclareLaunchArgument(
    #         name='scanner', default_value='/livox',
    #         description='Namespace for sample topics'
    #     ),
    
    # declare_targetns_cmd = DeclareLaunchArgument(
    #         name='target_ns', default_value='',
    #         description='Namespace for target topics'
    #     ),
    
    livox_driver = Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_ros2_params
        )
    
    pointcloud2laserscan = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [scanner, '/scan']),
            #             ('scan', [target_ns, '/scan'])],
            remappings=[('cloud_in', "/livox/lidar"),
                        ('scan', "/scan")],

            parameters=[{
                'target_frame': frame_id,
                'transform_tolerance': 0.01,
                'min_height': 1.0,
                'max_height': 2.0,
                'angle_min': -1.44,  # less than -M_PI/2
                'angle_max': 1.436,  # less than M_PI/2
                'angle_increment': 0.008,  #  2.88 /360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    return LaunchDescription([
        # declare_ns_cmd,
        # declare_targetns_cmd,
        livox_driver,
        pointcloud2laserscan,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
