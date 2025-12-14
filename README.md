# ERL Assignment 1 - ArUco Marker Navigation

This ROS 2 package implements a robot control system that spawns a robot in a Gazebo environment, detects ArUco markers, and navigates to them in ascending order based on their ID from ARUCO ORIGINAL dictionary.

## Project Overview

The main objective of this project is to control a robot to:
1.  **Scan**: Rotate in place to detect all visible ArUco markers in the environment.
2.  **Sort**: Once a sufficient number of markers (under assumption of 5 as the world contains 5 markers) are detected, sort them by their ID in ascending order.
3.  **Navigate**: Sequentially visit each marker, centering it in the camera view and approaching it until a specific size threshold is met.

## Features

-   **ArUco Marker Detection**: Uses OpenCV and the ArUco module to detect markers and estimate their position in the image.
-   **Visual Servoing**: Implements a proportional controller to align the robot with the target marker (angular control) and approach it (linear control).
-   **State Machine**: Simple state machine handling `SCANNING`, `NAVIGATING`, and `DONE` states.
-   **Simulation Integration**: Integrated with Gazebo and RViz for simulation and visualization.

## Prerequisites

-   **ROS 2** (Jazzy)
-   **Gazebo** (Gz Harmonic)
-   **Python 3**
-   **Dependencies**:
    -   `rclcpp`
    -   `sensor_msgs`
    -   `geometry_msgs`
    -   `nav_msgs`
    -   `cv_bridge`
    -   `image_transport`
    -   `ros_gz_sim`
    -   `ros_gz_bridge`
    -   `OpenCV`

## Installation

1.  **Clone the repository** into your ROS 2 workspace `src` folder:
    ```bash
    cd ~/exp_ws/src
    # Copy or clone the package here
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/exp_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the package**:
    ```bash
    colcon build --packages-select erl_assignment_1
    ```

4.  **Source the setup file**:
    ```bash
    source install/setup.bash
    ```

## Usage

To run the complete simulation, use the provided launch files for your desired robot model.

### 1. Two-Wheeled Robot (Deffirential Drive)
This is the default configuration using `mogi_bot`.

```bash
ros2 launch erl_assignment_1 task.launch.py
```

### 2. Four-Wheeled Robot (Skid-Steer)
This configuration uses the `mogi_bot_skid_steer` model.

```bash
ros2 launch erl_assignment_1 four_wheel_task.launch.py
```

### What this does:
-   Launches the Gazebo world with ArUco markers.
-   Spawns the `mogi_bot` robot model.
-   Starts the ROS-Gazebo bridge.
-   Opens RViz for visualization.
-   Starts the `aruco_control` node to begin the mission.

## Node Details: `aruco_control`

The core logic resides in the `aruco_control` node.

### Subscribed Topics
-   `/camera/image` (`sensor_msgs/msg/Image`): The raw image stream from the robot's camera.
-   `/odom` (`nav_msgs/msg/Odometry`): Robot odometry data.

### Published Topics
-   `/cmd_vel` (`geometry_msgs/msg/Twist`): Velocity commands to move the robot.
-   `/aruco_control/processed_image` (`sensor_msgs/msg/Image`): Debug image showing detections and status.

### Parameters
The node behavior can be configured via ROS parameters (see `aruco_control.cpp`):
-   `camera_topic`: Topic name for the camera (default: `/camera/image`).
-   `cmd_vel_topic`: Topic name for velocity commands (default: `/cmd_vel`).
-   `scan_speed`: Angular speed during the scanning phase (default: `0.4`).
-   `target_area`: The target area (in pixels) of the marker to consider it "reached" (default: `12000.0`).
-   `kp_ang`: Proportional gain for angular control (default: `0.002`).
-   `kp_lin`: Proportional gain for linear control (default: `0.00005`).

## Directory Structure

-   `aruco_control/`: C++ source code for the control node.
-   `launch/`: Launch files for the robot, world, and main task.
-   `urdf/`: URDF description of the `mogi_bot`.
-   `worlds/`: Gazebo world files.
-   `rviz/`: RViz configuration files.
