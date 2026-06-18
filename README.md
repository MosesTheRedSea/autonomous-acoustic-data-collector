#  ROS2 Robot Audition Suite
> Developed at the **Institute of Science Tokyo**, Japan

A research-oriented ROS2 framework for autonomous acoustic and spatial data collection using mobile robots.

![rover-room](https://github.com/MosesTheRedSea/ros2-robot-audition-suite/blob/rover-simulation/IMG_7073.jpg)

![rover](https://github.com/MosesTheRedSea/ros2-robot-audition-suite/blob/rover-simulation/IMG_6273.jpeg)


## Overview

A ROS2 Foxy-based framework for automated acoustic data collection in indoor environments. A mobile robot autonomously navigates to predefined waypoints within a room, records room impulse responses (RIR) at each location using a speaker and microphone array, and proceeds to the next waypoint automatically after each recording session.

The system is designed for experiments in:
- Embodied audition
- Spatial audio understanding
- Room impulse response (RIR) measurement
- Acoustic scene analysis
- Multimodal robotics datasets

## Motivation

Spatial audio understanding and embodied audition require large-scale paired acoustic datasets that are prohibitively expensive to collect manually. This project automates the collection of room impulse responses at defined spatial positions using a mobile robot, enabling systematic, repeatable acoustic measurements across indoor environments.

## Hardware

| Component | Model |
|---|---|
| Robot Platform | 4WDS Rover (VSTF WRC058) |
| Onboard Computer | Mini PC (Ubuntu 20.04) |
| Motor Controller | ESP32 via RS485 bus |
| LIDAR | YDLidar TG30 |
| Depth Camera | Orbbec Gemini 335L |
| Microphone Array | 16-channel USB mic array |
| Speaker | USB audio adapter |

### Node Descriptions

| Node | Language | Role |
|---|---|---|
| `waypoint` | C++ | Loads waypoints from YAML, sends goal poses, manages navigation state |
| `controller` | C++ | Proportional controller — subscribes to `/goal_pose`, publishes `/cmd_vel` |
| `collector` | C++ | State machine — triggers recording on waypoint arrival, auto-proceeds after completion |
| `handler` | C++ | ROS2 service server for manual operator override (`/proceed_to_next`, `/abort_session`) |
| `recorder` | C++ | Records depth/color/scan/pointcloud data to rosbag at each waypoint |
| `acoustic_recorder` | Python | Plays excitation signal, records mic array, computes and saves room impulse responses |

## Packages

```
ros2-robot-audition-suite/
├── src/
│   ├── audition_msgs/          # Custom message/service definitions
│   ├── audition_hardware/      # hardware_bridge (legacy, replaced by ROS1 bridge)
│   ├── audition_data_collector/# All collection nodes + launch files
│   │   ├── src/                # C++ nodes
│   │   ├── scripts/            # Python nodes (acoustic_recorder.py)
│   │   ├── launch/
│   │   │   ├── slam_map.launch.py      # LIDAR + SLAM + robot_state_publisher
│   │   │   └── rover_bringup.launch.py # Full collection stack
│   │   └── config/
│   │       ├── real_waypoints.yaml
│   │       └── acoustic_params.yaml
│   ├── audition_sim/           # Gazebo simulation
│   └── audition_bringup/       # Top-level launch coordination
├── scripts/
│   └── launch_rover.sh         # tmux session launcher for bridge chain
├── bridge_ws/                  # ros1_bridge workspace
└── ydlidar_ws/                 # YDLidar TG30 ROS2 driver workspace
```

## Quickstart — Simulation

```bash
git clone https://github.com/MosesTheRedSea/ros2-robot-audition-suite
cd ros2-robot-audition-suite

source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch audition_sim sim.launch.py
```

## Quickstart — Real Robot

### Prerequisites

- ROS2 Foxy on the rover's mini PC (Ubuntu 20.04)
- ROS1 Noetic installed alongside Foxy
- `ros1_bridge` built (see `bridge_ws/`)
- YDLidar TG30 driver built (see `ydlidar_ws/`)
- ESP32 flashed with `fwdsrover_x40a_common.ino` firmware

### Step 1 — SSH into the rover

```bash
ssh moses@<rover_ip>
```

### Step 2 — Launch the bridge chain (motors)

This starts `roscore`, `rosserial_python` (ESP32 comms), `topic_tools relay`, and `ros1_bridge` in a tmux session:

```bash
cd ~/moses-research/ros2-robot-audition-suite/scripts
./launch_rover.sh
tmux attach -t rover
```

Confirm window 1 (`Ctrl-b 1`) shows:
```
Setup subscriber on rover_twist [geometry_msgs/Twist]
```

### Step 3 — Manual teleop driving

In the teleop window (`Ctrl-b 4`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |

### Step 4 — Map the room with SLAM

While driving with teleop, run SLAM in a separate terminal to build a map:

```bash
source /opt/ros/foxy/setup.bash
source ~/moses-research/ros2-robot-audition-suite/install/setup.bash
ros2 launch audition_data_collector slam_map.launch.py
```

Drive around the entire room slowly. Once done, save the map (**before stopping SLAM**):

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/moses/room_map'}}"
```

Copy the map to your laptop to inspect it:

```bash
scp moses@<rover_ip>:~/room_map.pgm ~/Desktop/
scp moses@<rover_ip>:~/room_map.yaml ~/Desktop/
```

### Step 5 — Define waypoints

Open `room_map.pgm` and identify the x/y coordinates of each collection point. Update `src/audition_data_collector/config/real_waypoints.yaml`:

```yaml
waypoints:
  labels: ["position_1", "position_2", "position_3", "position_4"]
  x: [1.5, -2.5, -1.5,  2.5]
  y: [2.5,  1.5, -2.5, -1.5]
  yaw: [0.0, 0.0, 0.0, 0.0]
```

### Step 6 — Run autonomous data collection

```bash
colcon build --packages-select audition_data_collector
source install/setup.bash
ros2 launch audition_data_collector rover_bringup.launch.py
```

The robot will autonomously:
1. Navigate to each waypoint
2. Trigger acoustic recording (plays excitation signal, records mic array, computes RIR)
3. Auto-proceed to the next waypoint

Data is saved to `/home/moses/audition_bags/`.

## Manual Operator Override

Even in autonomous mode, an operator can manually abort a session:

```bash
ros2 service call /abort_session audition_msgs/srv/AbortSession \
  "{reason: 'manual stop'}"
```

Or force-proceed to the next waypoint:

```bash
ros2 service call /proceed_to_next audition_msgs/srv/ProceedToNext \
  "{proceed: true, operator_note: 'manual override'}"
```
