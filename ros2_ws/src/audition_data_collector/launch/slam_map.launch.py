import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description = subprocess.check_output(
        ['xacro', os.path.join(
            get_package_share_directory('audition_sim'),
            'urdf', 'robot.urdf.xacro')]
    ).decode('utf-8')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'frame_id': 'laser_link',
                'ignore_array': '',
                'baudrate': 512000,
                'lidar_type': 4,
                'device_type': 0,
                'sample_rate': 20,
                'abnormal_check_count': 4,
                'fixed_resolution': False,
                'reversion': True,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': True,
                'support_motor_dtr': False,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 30.0,
                'range_min': 0.1,
                'frequency': 10.0,
                'invalid_range_is_inf': False,
            }],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',
            }],
            output='screen'
        ),

    ])