# Trajectory Generation and Tracking

This ROS 2 package implements a complete pipeline for path smoothing, trajectory generation, and path tracking control for a differential drive robot (Turtlebot3).

## Features

1.  **Path Smoothing**: Converts discrete waypoints into a continuous B-Spline path.
2.  **Trajectory Generation**: Generates a time-parameterized trajectory with a Trapezoidal velocity profile.
3.  **Tracking Control**: Implements a Pure Pursuit controller (configurable to PID/Stanley) to follow the trajectory.
4.  **Simulation**: Includes a mock simulation environment to verify logic without heavy Gazebo dependencies.

## Prerequisites

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS Distro**: ROS 2 Humble
- **Dependencies**: `turtlebot3_gazebo`, `turtlebot3_description`, `rviz2`

## Installation

1.  **Navigate to your workspace**:
    ```bash
    cd ~/trajectory_ws1
    ```

2.  **Install dependencies**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the package**:
    ```bash
    colcon build --symlink-install --packages-select path_smoothing_control
    ```

4.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Execution

### Run the Simulation (Recommended)

This launch file starts the mock simulator, the control node, and RViz visualization.

```bash
ros2 launch path_smoothing_control mock_sim.launch.py
```

### What to Expect

1.  **RViz** will open showing:
    - **Red Line**: Original Waypoints.
    - **Green Line**: Smoothed B-Spline Path.
    - **Axis/Robot**: Moving along the path.
2.  **Terminal Logs** will show:
    - Use of B-Spline smoothing.
    - Trajectory generation details.
    - "Goal reached!" message upon completion.

## Configuration

You can modify parameters in `config/params.yaml`:
- `waypoints_x` / `waypoints_y`: Define the path.
- `max_velocity` / `max_acceleration`: Adjust the speed profile.
- `controller_type`: 0=PurePursuit, 1=Stanley, 2=PID.
