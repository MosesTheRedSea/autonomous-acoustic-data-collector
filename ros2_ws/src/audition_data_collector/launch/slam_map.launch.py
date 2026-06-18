from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser_link',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
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

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),

    ])
