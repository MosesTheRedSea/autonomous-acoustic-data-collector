import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    collector_pkg = get_package_share_directory('audition_data_collector')
    waypoints_config = os.path.join(collector_pkg, 'config', 'real_waypoints.yaml')
    acoustic_params = os.path.join(collector_pkg, 'config', 'acoustic_params.yaml')

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',   # LIDAR is usually a separate USB device from the ESP32
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

        TimerAction(period=5.0, actions=[

            Node(
                package='audition_data_collector',
                executable='waypoint',
                parameters=[waypoints_config, {'use_sim_time': False}],
                output='screen'
            ),

            Node(
                package='audition_data_collector',
                executable='collector',
                parameters=[{'use_sim_time': False}],
                output='screen'
            ),

            Node(
                package='audition_data_collector',
                executable='controller',
                parameters=[{'use_sim_time': False}],
                output='screen'
            ),

            Node(
                package='audition_data_collector',
                executable='handler',
                parameters=[{'use_sim_time': False}],
                output='screen'
            ),

            Node(
                package='audition_data_collector',
                executable='recorder',
                parameters=[{
                    'output_dir': '/home/moses/audition_bags',
                    'use_sim_time': False
                }],
                output='screen'
            ),

            Node(
                package='audition_data_collector',
                executable='acoustic_recorder.py',
                parameters=[acoustic_params, {'use_sim_time': False}],
                output='screen'
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                parameters=[{'use_sim_time': False}],
                output='screen'
            ),

        ]),

    ])
