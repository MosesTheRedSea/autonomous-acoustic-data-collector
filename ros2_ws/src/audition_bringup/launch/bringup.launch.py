  import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    collector_pkg = get_package_share_directory('audition_data_collector')
    sim_pkg = get_package_share_directory("audition_sim")

    waypoints_config = os.path.join(collector_pkg, 'config', 'real_waypoints.yaml')
    acoustic_params = os.path.join(sim_pkg, 'config', 'acoustic_params.yaml')

    return LaunchDescription([

        Node(
            package='audition_data_collector',
            executable='odom_integrator.py',
            name='odom_integrator',
            parameters=[{
                'start_x': 0.0,
                'start_y': 0.0,
                'start_yaw': 0.0,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
            }],
            output='screen'
        ),
        
        TimerAction(period=2.0, actions=[

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

            """
            Node(
                package='audition_data_collector',
                executable='recorder',
                parameters=[{
                    'output_dir': '/home/moses/audition_bags',
                    'use_sim_time': False
                }],
                output='screen'
            ),
            """

            Node(
                package='audition_data_collector',
                executable='acoustic_recorder.py',
                name='acoustic_recorder_node',
                parameters=[acoustic_params, {'use_sim_time': False}],
                output='screen'
            ),

        ]),

    ]) 
