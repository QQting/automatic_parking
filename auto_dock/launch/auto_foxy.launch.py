import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
	    #ExecuteProcess(
            #cmd=['ros2','run','tf2_ros','static_transform_publisher','0','0','0','0','0','0','map','laser'],
            #output='screen'),

        Node(
            package='laser_line_extraction',
            executable='laser_line_extraction_node',
            name='laser_line_extraction',
            output='screen',
            parameters=[{"frequency":50.0 ,
                        "frame_id":"laser_frame",
                        "scan_topic" :"scan",
                        "publish_markers": True,
                        "bearing_std_dev" :0.0015,
                        "range_std_dev" :0.01,
                        "least_sq_angle_thresh": 0.0001,
                        "least_sq_radius_thresh" :0.0001,
                        "max_line_gap" :0.5,
                        "min_line_length" :0.03,
                        "min_range" :0.2,
                        "min_split_dist":0.05,
                        "outlier_dist" :0.05,
                        "min_line_points": 5,
            }]),
        
        Node(
            package='auto_dock', 
            executable='pattern', 
            name='pattern_node',
            output='screen',
            parameters=[{"detect_angle_tolerance" : 0.2,
                        "group_dist_tolerance" :0.15,
                        "laser_frame_id":"laser_frame",
            }]
        ),

        Node(
            package='auto_dock',
           executable='controller',
            name='controller_node',
            output='screen',
            parameters=[{
                        "dist_to_dock":0.25,
                        "dist_to_center":0.05,

            }],
        ),
       
    ])
