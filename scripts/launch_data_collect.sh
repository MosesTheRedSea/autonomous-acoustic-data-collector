#!/bin/bash

SESSION="rover"

BRIDGE_WS=~/moses-research/ros2-robot-audition-suite/bridge_ws

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n roscore
tmux send-keys -t $SESSION:roscore "source /opt/ros/noetic/setup.bash && roscore" C-m

tmux new-window -t $SESSION -n rosserial
tmux send-keys -t $SESSION:rosserial "sleep 8 && source /opt/ros/noetic/setup.bash && rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200" C-m

tmux new-window -t $SESSION -n odom
tmux send-keys -t $SESSION:odom "sleep 11 && source /opt/ros/noetic/setup.bash && rosrun topic_tools relay /odom /odom" C-m

tmux new-window -t $SESSION -n bridge
tmux send-keys -t $SESSION:bridge "sleep 14 && source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && source $BRIDGE_WS/install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

tmux new-window -t $SESSION -n teleop
tmux send-keys -t $SESSION:teleop "sleep 17 && source /opt/ros/foxy/setup.bash && echo 'Use: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...'" C-m

tmux new-window -t $SESSION -n bringup
tmux send-keys -t $SESSION:bringup "sleep 17 && source /opt/ros/foxy/setup.bash && source ~/moses-research/ros2-robot-audition-suite/install/setup.bash && echo 'Run: ros2 launch audition_data_collector rover_bringup_fixedbox.launch.py'" C-m

echo "Launched CLEAN rover stack:"
echo "ROS2 /cmd_vel → ros1_bridge → ROS1 /cmd_vel → rosserial → ESP32"
echo "Attach with: tmux attach -t $SESSION"
echo "Switch windows with Ctrl-b n"
