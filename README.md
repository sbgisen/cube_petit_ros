# CubePetit ROS


Go to [Japanese Page](README_ja.md)

CubePetit is a small ROS2-based mobile robot platform for learning, prototyping, and experimenting with Embodied AI.

## Overview

CubePetit is a compact autonomous mobile robot designed to live and move in human environments.
This repository provides ROS2 packages for simulation, bringup, teleoperation, speech interaction, facial animation, and Python APIs.

## Repositories

- [cube_petit](https://github.com/sbgisen/cube_petit) : More about Cube petit & DIY Kit
- [cube_petit_cad](https://github.com/sbgisen/cube_petit_cad) : Cube petit's CAD
- [cube_petit_ros](https://github.com/sbgisen/cube_petit_ros) : This Repository
- [cube_petit_interaction](https://github.com/sbgisen/cube_petit_interaction) : Base packages for robot interaction
- [cube_petit_scenario](https://github.com/sbgisen/cube_petit_scenario) : Demo scenarios built on cube_petit_interaction

## Features

- ROS2-based robot software stack
- Gazebo simulation
- Teleoperation with game controller
- 2D LiDAR bringup
- Speech-to-text and text-to-speech
- Facial animation
- Python API for speech and LLM-based interaction
- Hardware bringup for the real robot

## Demo

- Video: [YouTube](https://www.youtube.com/playlist?list=PL509ZQjTHPYecUfyNaroISz6ZV1QCh2k4)
- Exhibition updates: [X / Twitter@Cube_petit_2022](https://x.com/Cube_petit_2022)
- Product / project page: [Website](https://www.ros-sier.com/case/hardware/cubepetit)

## Supported Environment

| Item | Supported |
|---|---|
| OS | Ubuntu 18.04 / 20.04 / 22.04 / 24.04 |
| ROS | Melodic / Noetic / Humble / Jazzy |
| Simulator | Gazebo / gz-sim |
| Robot | CubePetit v1 / v2 / v3 |

## Repository Structure

- `cube_petit_ros`: meta package
- `cube_petit_description`: URDF / xacro model
- `cube_petit_gazebo`: simulation
- `cube_petit_bringup`: real robot bringup
- `cube_petit_hardware_interface`: motor interface
- `cube_petit_facial_animation`: face animation
- `cube_petit_speech_to_text`: speech recognition
- `cube_petit_text_to_speech`: speech synthesis
- `cube_petit_python_api`: Python API for speech and LLM interaction

## Quick Start

### 0. Install dependencies

```bash
sudo apt update
sudo apt install python3-vcstool python3-rosdep
sudo rosdep init  # skip if already initialized
```

### 1. Create workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone this repository

```bash
git clone https://github.com/sbgisen/cube_petit_ros.git
cd ~/ros2_ws
vcs import src < src/cube_petit_ros/cube_petit_ros.repos
```

### 3. Install dependencies

```bash
rosdep update
rosdep install -r -y -i --from-paths src
```

### 4. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Launch simulation

```bash
ros2 launch cube_petit_gazebo cube_petit_gazebo.launch.py
```

### 6. Real Robot Setup

Coming soon.

## Branches

| Branch   | ROS version | Status             | Note        |
|----------|-------------|--------------------|-------------|
| develop  | latest      | stable             | recommended |
| jazzy    | ROS2 Jazzy  | active development |             |
| humble   | ROS2 Humble | maintained         |             |
| noetic   | ROS1 Noetic | legacy             |             |
| melodic  | ROS1 Melodic| legacy             |             |

## Roadmap
- □ ROS2 Jazzy support
- □ New Gazebo / gz-sim support
- □ Navigation2 examples
- □ LLM conversation examples
- □ Multi-robot examples
- □ Documentation for custom hardware

## Contributing

Issues and pull requests are welcome.
For major changes, please open an issue first to discuss what you would like to change.

## License

Apache License 2.0
