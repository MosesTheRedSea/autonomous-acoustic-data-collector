from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess


def generate_launch_description():

    robot_description = subprocess.check_output([
        'xacro',
        os.path.join(
            get_package_share_directory('audition_sim'),
            'urdf',
            'robot.urdf.xacro'
        )
    ]).decode('utf-8')

    return LaunchDescription([

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }],
            output='screen'
        ),

        # IMPORTANT: LiDAR node
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar',
            parameters=[{
                'frame_id': 'laser_link',
                'port': '/dev/ttyUSB1',
                'baudrate': 512000,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 30.0,
                'range_min': 0.1,
                'frequency': 10.0,
                'intensity': True,
            }],
            output='screen'
        ),

        # SLAM Toolbox (FIXED)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',

                # IMPORTANT stability flags
                'use_scan_matching': True,
                'use_odom': False,   # critical for your setup
            }],
            output='screen'
        ),

        # RViz optional
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])