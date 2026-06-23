#!/bin/bash
# launch_rover.sh
# Brings up the full ROS2 -> ROS1 -> ESP32 bridge chain in one tmux session,
# including bidirectional odometry relay for the fixed 5x5m box collection.
#
# Usage:
#   chmod +x launch_rover.sh
#   ./launch_rover.sh
#
# Attach later with:  tmux attach -t rover
# Kill everything with: tmux kill-session -t rover

SESSION="rover"

# Adjust this path if your bridge workspace lives elsewhere
BRIDGE_WS=~/moses-research/ros2-robot-audition-suite/bridge_ws

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n roscore
tmux send-keys -t $SESSION:roscore "source /opt/ros/noetic/setup.bash && roscore" C-m

tmux new-window -t $SESSION -n rosserial
tmux send-keys -t $SESSION:rosserial \
  "sleep 8 && source /opt/ros/noetic/setup.bash && rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200" C-m

tmux new-window -t $SESSION -n relay
tmux send-keys -t $SESSION:relay \
  "sleep 11 && source /opt/ros/noetic/setup.bash && rosrun topic_tools relay /cmd_vel /rover_twist" C-m

tmux new-window -t $SESSION -n odom_relay
tmux send-keys -t $SESSION:odom_relay \
  "sleep 11 && source /opt/ros/noetic/setup.bash && rosrun topic_tools relay /rover_odo /rover_twist_odo" C-m

tmux new-window -t $SESSION -n bridge
tmux send-keys -t $SESSION:bridge \
  "sleep 14 && source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && source $BRIDGE_WS/install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

tmux new-window -t $SESSION -n teleop
tmux send-keys -t $SESSION:teleop \
  "sleep 17 && source /opt/ros/foxy/setup.bash && echo 'Optional manual drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard'" C-m

tmux new-window -t $SESSION -n bringup
tmux send-keys -t $SESSION:bringup \
  "sleep 17 && source /opt/ros/foxy/setup.bash && source ~/moses-research/ros2-robot-audition-suite/install/setup.bash && echo 'Run: ros2 launch audition_data_collector rover_bringup_fixedbox.launch.py'" C-m

echo "Launched tmux session '$SESSION' with windows: roscore, rosserial, relay, odom_relay, bridge, teleop, bringup"
echo "Attach with: tmux attach -t $SESSION"
echo "Switch windows with Ctrl-b then window number (0-6), or Ctrl-b n for next"
