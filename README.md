#  Autonomous Acoustic Data Collector

[![DAMP Core](https://img.shields.io/badge/Project-DAMP_Core-blue)](https://github.com/MosesTheRedSea/damp-core)
[![Sim2Real](https://img.shields.io/badge/Project-Sim2Real-purple)](https://github.com/MosesTheRedSea/occlunet-sim2real)
[![Active Path Planning](https://img.shields.io/badge/Project-Path_Planning-green)](https://github.com/MosesTheRedSea/occulnet-active-search)


> Developed at the **Institute of Science Tokyo**, Japan

A research-oriented ROS2 framework for autonomous acoustic and spatial data collection using mobile robots.

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



## Simulation

```bash
git clone https://github.com/MosesTheRedSea/ros2-robot-audition-suite
cd ros2-robot-audition-suite

source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch audition_sim sim.launch.py
```

## Run Autonomous Data Collection

#### SSH into the rover

```bash
ssh moses@<rover_ip>
```

```bash
colcon build --packages-select audition_data_collector
source /opt/ros/foxy/setup.bash

source install/setup.bash
ros2 launch audition_bringup bringup.launch.py
```


####  Launch the bridge chain (motors)

This starts `roscore`, `rosserial_python` (ESP32 comms), `topic_tools relay`, and `ros1_bridge` in a tmux session:

```bash

source /opt/ros/foxy/setup.bash
source install/setup.bash

cd scripts

./launch_rover.sh
tmux attach -t rover
```




