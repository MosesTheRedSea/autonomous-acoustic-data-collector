from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'frame_id': 'laser_link',
                'ignore_array': '',
                'baudrate': 230400,          # CONFIRM: X4=128000, G2/G4=230400, TG30=512000
                'lidar_type': 1,              # 1 = TYPE_TRIANGLE (most YDLidar models)
                'device_type': 0,             # 0 = YDLIDAR_TYPE_SERIAL
                'sample_rate': 9,
                'abnormal_check_count': 4,
                'fixed_resolution': False,
                'reversion': True,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': False,
                'support_motor_dtr': True,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 64.0,
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