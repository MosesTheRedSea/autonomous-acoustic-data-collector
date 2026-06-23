import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/noetic/setup.bash && '
                'rosrun rosserial_python serial_node.py '
                '_port:=/dev/ttyUSB0 _baud:=115200'
            ],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'source /opt/ros/noetic/setup.bash && '
                        'rosrun topic_tools relay /cmd_vel /rover_twist'
                    ],
                    output='screen'
                )
            ]
        ),


        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'source /opt/ros/noetic/setup.bash && '
                        'source /opt/ros/foxy/setup.bash && '
                        'source ~/moses-research/ros2-robot-audition-suite/bridge_ws/install/setup.bash && '
                        'ros2 run ros1_bridge dynamic_bridge --bridge-all-topics'
                    ],
                    output='screen'
                )
            ]
        ),

        
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
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
            ]
        ),

        
        TimerAction(
            period=12.0,
            actions=[

                Node(
                    package='audition_data_collector',
                    executable='waypoint',
                    output='screen'
                ),

                Node(
                    package='audition_data_collector',
                    executable='collector',
                    output='screen'
                ),

                Node(
                    package='audition_data_collector',
                    executable='controller',
                    output='screen'
                ),

                Node(
                    package='audition_data_collector',
                    executable='handler',
                    output='screen'
                ),

                Node(
                    package='audition_data_collector',
                    executable='acoustic_recorder',
                    parameters=[{
                        'excitation_path': '/home/moses/excitation.wav',
                        'output_dir': '/home/moses/audition_bags/acoustic',
                        'channels': 16,
                        'repeat': 8,
                        'sleep_duration': 3,
                        'start_sample': 4900,
                        'end_sample': 6000,
                        'mic_device': 5,
                        'speaker_device': 8,
                    }],
                    output='screen'
    )           ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen'
                ),
            ]
        ),
    ])
