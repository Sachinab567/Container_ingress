# ROS2 Autonomous Container Ingress/Egress

This project implements an autonomous navigation system for the **Vegam** robot (a differential drive mobile robot) to navigate into a container, perform maneuvers, and exit while maintaining perfect centering and orientation.

## Key Features
- **Custom FSM Controller**: A pure Python-based controller (no Nav2 dependency) managing the cycle:
  - `APPROACH`: Detects the container and aligns for entry.
  - `INSIDE`: Centered navigation using LiDAR-based wall distance monitoring.
  - `TURN_IN`: High-precision 180° spin-in-place using odometry yaw feedback.
  - `EXIT`: Parallel exit navigation with a "Yaw-Lock" mechanism to prevent drift.
  - `TURN_OUT`: Final 180° spin to face the container after successful exit.
- **URDF & Gazebo**:
  - Accurate physics tuning (friction optimization for spin-in-place).
  - Fixed sensor placements (LiDAR and IMU) for realistic simulation.
- **High Performance**: Optimized for 0.35 m/s drive speed with proportional control for smooth rotation.

## Prerequisites
- **OS**: Ubuntu 22.04
- **Middleware**: ROS2 Humble
- **Simulation**: Gazebo (classic)

## Installation
```bash
# Clone the repository into your ROS2 workspace
cd ~/<work_space>/src
git clone <repository_url> .

# Build the workspace
cd ~/<work_space>
colcon build --packages-select Vegam_description container_ingress

# Source the setup script
source install/setup.bash
```

## Usage
To launch the Gazebo simulation and the autonomous controller:
```bash
ros2 launch container_ingress ingress.launch.py
```
Wait for the Gazebo UI to load, then click the **Play** button. The robot will automatically:
1. Drive towards the container.
2. Center itself between the walls.
3. Reach the back wall and spin 180°.
4. Exit the container straight.
5. Spin 180° again to face the entrance.

## Package Overview
- `Vegam_description`: Contains the robot URDF (Xacro), meshes, and Gazebo plugins.
- `container_ingress`: Contains the controller logic (`ingress_controller.py`), wall/container world files, and launch scripts.

## Robot Specs (Vegam)
- **Drive**: Differential Drive (2 driven rear wheels, 2 front casters).
- **Sensors**: 360° LiDAR, IMU.
- **Controller Type**: Pure Pursuit / PID-based Centering.
