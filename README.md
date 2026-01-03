# Trajectory Generation and Tracking

A comprehensive ROS 2 package implementing path smoothing, trajectory generation, and path tracking control for differential drive robots (TurtleBot3). This system provides multiple controller options and a lightweight simulation environment for rapid testing.

---

## 📋 Table of Contents

- [Features](#features)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Launch Instructions](#launch-instructions)
- [Algorithm Selection](#algorithm-selection)
- [Configuration](#configuration)
- [Testing](#testing)
- [Design & Algorithms](#design-choices--algorithms)
- [Obstacle Avoidance](#obstacle-avoidance-extension)
- [Real Robot Deployment](#deployment-to-real-robot)
- [Project Structure](#project-structure)

---

## ✨ Features

- **Path Smoothing**: Converts discrete waypoints into continuous C² B-Spline paths
- **Trajectory Generation**: Time-parameterized trajectories with trapezoidal velocity profiles
- **Multiple Controllers**: Pure Pursuit, Stanley, and PID controllers
- **Mock Simulation**: Lightweight Python-based simulator for rapid testing
- **Comprehensive Testing**: 18 automated unit tests covering all components
- **RViz Visualization**: Real-time path and robot state visualization

---

## 🚀 Quick Start

```bash
# Clone and navigate to workspace
cd ~/trajectory_ws-main

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --packages-select path_smoothing_control

# Source
source install/setup.bash

# Launch simulation
ros2 launch path_smoothing_control mock_sim.launch.py
```

**Expected Output**: RViz opens showing the robot following a smoothed path with real-time visualization.

---

## 📦 Installation

### Prerequisites

| Requirement | Version/Details |
|------------|----------------|
| **Operating System** | Ubuntu 22.04 (Jammy) |
| **ROS Distribution** | ROS 2 Humble |
| **Python** | 3.10+ |
| **Build Tools** | colcon, cmake |

### System Dependencies

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs

# Install additional dependencies
sudo apt install python3-pip python3-colcon-common-extensions
```

### Workspace Setup

#### Option 1: Fresh Installation

```bash
# Create workspace
mkdir -p ~/trajectory_ws-main/src
cd ~/trajectory_ws-main

# Clone repository (if from git)
# git clone <repository-url> src/path_smoothing_control

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install --packages-select path_smoothing_control

# Source workspace
source install/setup.bash

# Add to bashrc for convenience (optional)
echo "source ~/trajectory_ws-main/install/setup.bash" >> ~/.bashrc
```

#### Option 2: Existing Workspace

```bash
# Navigate to existing workspace
cd ~/trajectory_ws-main

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build only this package
colcon build --symlink-install --packages-select path_smoothing_control

# Source
source install/setup.bash
```

### Verify Installation

```bash
# Check if package is found
ros2 pkg list | grep path_smoothing_control

# Check available launch files
ros2 launch path_smoothing_control --show-args mock_sim.launch.py
```

---

## 🎮 Launch Instructions

### 1. Mock Simulation (Recommended for Testing)

**Purpose**: Lightweight simulation with RViz visualization, no Gazebo required.

```bash
ros2 launch path_smoothing_control mock_sim.launch.py
```

**What Happens**:
- Starts mock simulator node (differential drive kinematics)
- Launches path smoothing and control node
- Opens RViz with pre-configured visualization
- Robot follows the configured waypoint path

**Visualization**:
- 🔴 **Red Line**: Original waypoints
- 🟢 **Green Line**: Smoothed B-Spline path
- 🔵 **Blue Arrows**: Trajectory poses with timestamps
- 🤖 **Robot Model**: TurtleBot3 following the path

**Terminal Output**:
```
[INFO] [path_smoothing_node]: Path smoothing complete: 200 points
[INFO] [path_smoothing_node]: Trajectory generated: 15.3s duration
[INFO] [path_smoothing_node]: Using Pure Pursuit controller
[INFO] [path_smoothing_node]: Tracking error: 0.05m
[INFO] [path_smoothing_node]: Goal reached!
```

### 2. Control Node Only (For Real Robot)

**Purpose**: Run only the control node, expecting odometry from real robot.

```bash
ros2 launch path_smoothing_control node_only.launch.py
```

**Requirements**:
- Real robot publishing `/odom` topic
- Robot subscribing to `/cmd_vel` topic

**Use Case**: Deploy to TurtleBot3 or other differential drive robot.

### 3. Gazebo Simulation (Full Physics)

**Purpose**: High-fidelity simulation with Gazebo physics engine.

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Launch Gazebo simulation
ros2 launch path_smoothing_control gazebo_world.launch.py
```

**What Happens**:
- Spawns TurtleBot3 in Gazebo world
- Launches control node
- Opens RViz for visualization

**Note**: Requires more computational resources than mock simulation.

### 4. Custom Launch with Parameters

Override parameters at launch time:

```bash
ros2 launch path_smoothing_control mock_sim.launch.py \
  max_velocity:=0.2 \
  controller_type:=1 \
  lookahead_distance:=0.5
```

---

## 🧠 Algorithm Selection

This package provides **three controller algorithms**, each optimized for different scenarios.

### Controller Comparison

| Controller | Best For | Pros | Cons | Tuning Difficulty |
|-----------|----------|------|------|------------------|
| **Pure Pursuit** | Smooth paths, gentle curves | Simple, robust, smooth output | Less precise on tight curves | ⭐ Easy |
| **Stanley** | High-speed, precise tracking | Excellent lateral error correction | Requires heading alignment | ⭐⭐ Moderate |
| **PID** | Fine-tuned performance | Eliminates steady-state error | Requires careful tuning | ⭐⭐⭐ Hard |

### How to Select a Controller

#### Method 1: Configuration File

Edit [`config/params.yaml`](file:///home/newuser/trajectory_ws-main/src/path_smoothing_control/config/params.yaml):

```yaml
path_smoothing_node:
  ros__parameters:
    controller_type: 0  # 0=Pure Pursuit, 1=Stanley, 2=PID
```

#### Method 2: Launch Parameter

```bash
ros2 launch path_smoothing_control mock_sim.launch.py controller_type:=1
```

#### Method 3: Runtime Parameter Update

```bash
ros2 param set /path_smoothing_node controller_type 2
```

### Controller Details

#### 1. Pure Pursuit (Default - Type 0)

**Algorithm**: Geometric path tracking using lookahead point.

**Key Parameters**:
```yaml
controller_type: 0
lookahead_distance: 0.35  # meters (larger = smoother, smaller = more aggressive)
```

**When to Use**:
- ✅ Smooth, curved paths
- ✅ Moderate speeds (< 0.5 m/s)
- ✅ When simplicity is preferred
- ❌ Not ideal for sharp turns or high precision

**Tuning Guide**:
- Increase `lookahead_distance` if robot oscillates
- Decrease `lookahead_distance` for tighter path following

#### 2. Stanley Controller (Type 1)

**Algorithm**: Heading error + cross-track error with velocity-adaptive gain.

**Key Parameters**:
```yaml
controller_type: 1
kp_angular: 2.0  # Heading error gain
kp_linear: 1.0   # Cross-track error gain
```

**When to Use**:
- ✅ High-speed navigation
- ✅ Precise lateral error correction
- ✅ Straight paths with occasional curves
- ❌ Not ideal for very low speeds (< 0.1 m/s)

**Tuning Guide**:
- Increase `kp_angular` for faster heading correction
- Increase `kp_linear` for tighter path following

#### 3. PID Controller (Type 2)

**Algorithm**: Classic PID control on position and heading errors.

**Key Parameters**:
```yaml
controller_type: 2
kp_linear: 1.0   # Proportional gain (position)
kp_angular: 2.0  # Proportional gain (heading)
ki: 0.0          # Integral gain (eliminates steady-state error)
kd: 0.1          # Derivative gain (dampens oscillations)
```

**When to Use**:
- ✅ Need to eliminate steady-state error
- ✅ Custom performance requirements
- ✅ When you have time to tune parameters
- ❌ Not recommended for beginners

**Tuning Guide** (Ziegler-Nichols Method):
1. Set `ki=0`, `kd=0`
2. Increase `kp` until oscillations occur
3. Set `kp = 0.6 * kp_critical`
4. Set `ki = 2 * kp / T_oscillation`
5. Set `kd = kp * T_oscillation / 8`

### Path Smoothing Algorithm

**Algorithm**: Cubic B-Spline with clamped uniform knot vectors.

**Why B-Splines?**
- ✅ C² continuity (smooth curvature)
- ✅ Local control (changing one waypoint affects only nearby curve)
- ✅ Guaranteed to pass through first and last waypoints
- ✅ Efficient evaluation using De Boor's algorithm

**Parameters**:
```yaml
smoothing_samples: 200  # Number of points to sample from B-Spline
```

### Trajectory Generation Algorithm

**Algorithm**: Trapezoidal velocity profile with time parameterization.

**Profile Phases**:
1. **Acceleration**: Linear ramp from 0 to `max_velocity`
2. **Constant Velocity**: Maintain `max_velocity`
3. **Deceleration**: Linear ramp to 0

**Parameters**:
```yaml
max_velocity: 0.15      # m/s
max_acceleration: 0.3   # m/s²
```

**Automatic Triangle Profile**: For short paths where `max_velocity` cannot be reached, automatically switches to acceleration → deceleration.

---

## ⚙️ Configuration

### Parameter File: `config/params.yaml`

```yaml
path_smoothing_node:
  ros__parameters:
    # Path Definition
    waypoints_x: [0.0, 2.0, 4.0, 4.0, 2.0, 0.0, 0.0]
    waypoints_y: [0.0, 0.0, 1.0, 3.0, 4.0, 4.0, 0.5]
    
    # Path Smoothing
    smoothing_samples: 200
    
    # Trajectory Generation
    max_velocity: 0.15        # m/s
    max_acceleration: 0.3     # m/s²
    
    # Controller Selection
    controller_type: 0        # 0=Pure Pursuit, 1=Stanley, 2=PID
    
    # Pure Pursuit Parameters
    lookahead_distance: 0.35  # meters
    
    # PID/Stanley Parameters
    kp_linear: 1.0
    kp_angular: 2.0
    ki: 0.0
    kd: 0.1
    
    # Robot Constraints
    max_v_lin: 0.22          # Maximum linear velocity (m/s)
    max_v_ang: 2.84          # Maximum angular velocity (rad/s)
    
    # Control Loop
    control_frequency: 20.0   # Hz
```

### Modifying Waypoints

**Method 1**: Edit `config/params.yaml` and rebuild:
```yaml
waypoints_x: [0.0, 1.0, 2.0, 3.0]
waypoints_y: [0.0, 1.0, 0.0, 1.0]
```

**Method 2**: Create custom parameter file:
```bash
ros2 launch path_smoothing_control mock_sim.launch.py \
  params_file:=/path/to/custom_params.yaml
```

---

## 🧪 Testing

## Testing

### Running Unit Tests

The package includes comprehensive unit tests for all core components:

```bash
# Build with tests enabled (default)
colcon build --packages-select path_smoothing_control

# Run all tests
colcon test --packages-select path_smoothing_control

# View test results
colcon test-result --verbose
```

### Test Coverage

- **Path Smoother Tests** (`test_path_smoother`): 5 tests covering B-Spline generation, endpoint interpolation, and edge cases
- **Trajectory Generator Tests** (`test_trajectory_generator`): 5 tests covering velocity profiles, time parameterization, and constraints
- **Controller Tests** (`test_controller`): 8 tests covering all three controller types (Pure Pursuit, Stanley, PID)

**Total: 18 automated tests**

### Running Individual Tests

After building, you can run individual test executables:

```bash
./build/path_smoothing_control/test_path_smoother
./build/path_smoothing_control/test_trajectory_generator
./build/path_smoothing_control/test_controller
```

## Configuration

You can modify parameters in `config/params.yaml`:
- `waypoints_x` / `waypoints_y`: Define the path.
- `max_velocity` / `max_acceleration`: Adjust the speed profile.
- `controller_type`: 0=PurePursuit, 1=Stanley, 2=PID.
- `lookahead_distance`: Pure Pursuit lookahead parameter.
- `kp_linear`, `kp_angular`, `ki`, `kd`: Controller gains.

## Design Choices & Algorithms

### Path Smoothing: B-Spline Algorithm

**Why I Chose B-Splines**

I selected B-Splines for path smoothing because they provide C² continuity (continuous curvature), which is essential for differential drive robots with limited angular acceleration. Unlike Bezier curves, B-Splines offer local control—changing one control point affects only a local region of the curve, making them more predictable and easier to work with.

A critical requirement was ensuring the robot starts and ends at the exact waypoint positions. To achieve this, I implemented clamped uniform knot vectors, which guarantee that the curve passes through the first and last control points.

**Implementation Details**

I implemented cubic B-Splines (degree 3) for optimal smoothness. However, I added logic to automatically reduce the degree for paths with fewer than 4 waypoints, ensuring the algorithm remains robust for edge cases.

The continuous B-Spline curve is sampled at 200 points (configurable via parameters) to generate the discrete trajectory. For efficient evaluation, I used De Boor's algorithm, which recursively computes the B-Spline basis functions.

### Trajectory Generation: Trapezoidal Velocity Profile

**Why I Chose Trapezoidal Profiles**

I implemented a trapezoidal velocity profile because it provides smooth, constant acceleration and deceleration phases, which reduce mechanical stress on the robot and improve tracking accuracy. This profile naturally respects the robot's maximum velocity and acceleration constraints while being computationally efficient.

For most paths, trapezoidal profiles are near time-optimal, striking a good balance between performance and simplicity.

**Implementation Details**

My implementation consists of three phases:
1. **Acceleration phase**: Linear velocity ramps from 0 to `max_velocity`
2. **Constant velocity phase**: Maintains `max_velocity` 
3. **Deceleration phase**: Linear velocity ramps down to 0

For short paths where the robot cannot reach `max_velocity`, I automatically switch to a triangle profile (acceleration immediately followed by deceleration).

To generate timestamps, I integrated the inverse velocity profile using trapezoidal numerical integration, ensuring accurate time parameterization along the entire trajectory.

### Trajectory Tracking Controllers

I implemented three different controller types, each with distinct trade-offs:

#### 1. Pure Pursuit (Default)

I chose Pure Pursuit as the default controller because it's particularly well-suited for differential drive robots and smooth paths with gentle curves.

**Why I like this approach**:
- Simple and robust implementation
- Naturally handles path following without complex calculations
- Produces smooth control outputs
- Well-matched to differential drive kinematics

**How it works**: The controller computes the angular velocity needed to reach a lookahead point on the path. The lookahead distance parameter determines how "aggressive" the tracking behavior is—larger values produce smoother but less accurate tracking, while smaller values are more aggressive but may cause oscillations.

#### 2. Stanley Controller

I included the Stanley controller because it excels at precise lateral error correction, making it ideal for applications requiring tight path following.

**Advantages of this approach**:
- Explicitly minimizes cross-track error
- Performs well at higher speeds
- Proven track record in autonomous vehicles (used in Stanford's DARPA Grand Challenge entry)

**How it works**: The controller combines heading error and cross-track error with a velocity-adaptive gain, providing more aggressive correction when needed.

#### 3. PID Controller

I implemented a classic PID controller for users who need fine-tuned performance and want to eliminate steady-state errors through integral action.

**Benefits**:
- Integral term eliminates steady-state errors
- Derivative term dampens oscillations
- Highly customizable through gain tuning

**How it works**: Classic PID control applied to both position and heading errors, requiring careful tuning of `kp_linear`, `kp_angular`, `ki`, and `kd` gains.

### Architectural Decisions

**Modular Library Design**

I designed the system with modularity as a core principle. Path smoothing, trajectory generation, and control are implemented as separate libraries, which enables independent testing and makes the code reusable in other projects. I maintained clean separation between interfaces (`.hpp` files) and implementations (`.cpp` files), and ensured each component has dedicated unit tests.

**ROS 2 Integration**

I leveraged ROS 2's parameter server for all configuration, allowing easy runtime modification without recompilation. I used standard ROS 2 messages (`nav_msgs/Path`, `geometry_msgs/Twist`, `nav_msgs/Odometry`) to ensure compatibility with existing ROS 2 tools and workflows. All paths are published as `nav_msgs/Path` for seamless RViz visualization.

**Mock Simulation**

To enable rapid testing without the overhead of Gazebo, I created a lightweight Python-based mock simulator. It implements basic differential drive kinematics for pose integration and broadcasts TF transforms for RViz visualization. This approach significantly speeds up the development and testing cycle



---

## 🚧 Obstacle Avoidance Extension (Extra Credit)

This section outlines how to extend the system with obstacle avoidance capabilities for real-world deployment.

### Overview

The current system assumes a static, obstacle-free environment. For real-world applications, we need **reactive obstacle avoidance** that can handle:
- Static obstacles (walls, furniture, barriers)
- Dynamic obstacles (people, other robots)
- Unknown environments (no prior map)

### Recommended Approach: Hybrid DWA + Local Replanning

I recommend a **two-layer approach** combining the strengths of reactive and deliberative planning:

```
┌─────────────────────────────────────────────────────────────┐
│                    Global Path Planning                      │
│  Waypoints → B-Spline Smoothing → Trajectory Generation     │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│              Local Obstacle Avoidance Layer                  │
│  ┌─────────────────┐         ┌──────────────────┐          │
│  │  DWA Planner    │◄────────┤ Obstacle Detector│          │
│  │  (Primary)      │         │  (LiDAR/Camera)  │          │
│  └────────┬────────┘         └──────────────────┘          │
│           │                                                  │
│           │ If stuck                                         │
│           ▼                                                  │
│  ┌─────────────────┐                                        │
│  │ Local Replanner │                                        │
│  │  (Fallback)     │                                        │
│  └────────┬────────┘                                        │
└───────────┼──────────────────────────────────────────────────┘
            │
            ▼
    ┌───────────────┐
    │  Controller   │
    │  (Pure Pursuit│
    │   /Stanley)   │
    └───────┬───────┘
            │
            ▼
        cmd_vel
```

### Implementation Roadmap

#### Phase 1: Obstacle Detection (2-3 hours)

**Goal**: Process sensor data and maintain obstacle map.

**New ROS Node**: `obstacle_detector_node`

**Subscriptions**:
- `/scan` (sensor_msgs/LaserScan) - LiDAR data
- `/camera/depth/points` (sensor_msgs/PointCloud2) - Depth camera (optional)

**Publications**:
- `/obstacles` (sensor_msgs/PointCloud2) - Detected obstacles
- `/costmap` (nav_msgs/OccupancyGrid) - Local costmap

**Implementation**:

```cpp
// include/path_smoothing_control/obstacle_detector.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ObstacleDetector {
public:
  struct Obstacle {
    double x, y;           // Position in robot frame
    double distance;       // Distance from robot
    double angle;          // Angle from robot heading
  };

  // Convert LaserScan to obstacle list
  std::vector<Obstacle> processLaserScan(
    const sensor_msgs::msg::LaserScan& scan);
  
  // Generate local costmap (grid-based representation)
  nav_msgs::msg::OccupancyGrid generateCostmap(
    const std::vector<Obstacle>& obstacles,
    double resolution = 0.05,  // 5cm grid cells
    double width = 5.0,        // 5m x 5m costmap
    double height = 5.0);
  
  // Check if a point is collision-free
  bool isCollisionFree(double x, double y, double safety_radius = 0.3);

private:
  std::vector<Obstacle> obstacles_;
  nav_msgs::msg::OccupancyGrid costmap_;
};
```

**Key Parameters**:
```yaml
obstacle_detector:
  ros__parameters:
    safety_radius: 0.3        # meters (robot radius + buffer)
    max_obstacle_distance: 3.0 # meters (ignore far obstacles)
    costmap_resolution: 0.05   # meters per cell
    costmap_size: 5.0          # meters (local costmap size)
```

#### Phase 2: Dynamic Window Approach (4-6 hours)

**Goal**: Generate collision-free velocity commands.

**Algorithm**: Sample velocity space (v, ω) and simulate forward trajectories.

**Implementation**:

```cpp
// include/path_smoothing_control/dwa_planner.hpp
#pragma once
#include "obstacle_detector.hpp"
#include <geometry_msgs/msg/twist.hpp>

class DWAPlanner {
public:
  struct DWAConfig {
    double max_v = 0.22;          // m/s
    double max_w = 2.84;          // rad/s
    double max_accel_v = 0.5;     // m/s²
    double max_accel_w = 3.0;     // rad/s²
    double v_resolution = 0.02;   // velocity sampling resolution
    double w_resolution = 0.1;    // angular velocity sampling
    double predict_time = 2.0;    // seconds to simulate forward
    double dt = 0.1;              // simulation timestep
    
    // Cost function weights
    double heading_weight = 1.0;   // Align with goal
    double clearance_weight = 2.0; // Stay away from obstacles
    double velocity_weight = 0.5;  // Prefer higher velocities
  };

  // Compute safe velocity given desired velocity and obstacles
  geometry_msgs::msg::Twist computeSafeVelocity(
    const geometry_msgs::msg::Twist& desired_vel,
    const std::vector<ObstacleDetector::Obstacle>& obstacles,
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Point& goal_point);

private:
  // Generate dynamic window (feasible velocities)
  std::vector<std::pair<double, double>> generateDynamicWindow(
    double current_v, double current_w, double dt);
  
  // Simulate trajectory for (v, w) pair
  std::vector<geometry_msgs::msg::Pose> simulateTrajectory(
    const geometry_msgs::msg::Pose& start_pose,
    double v, double w, double duration, double dt);
  
  // Check if trajectory collides with obstacles
  bool isTrajectoryCollisionFree(
    const std::vector<geometry_msgs::msg::Pose>& trajectory,
    const std::vector<ObstacleDetector::Obstacle>& obstacles,
    double safety_radius);
  
  // Compute cost for a trajectory
  double computeCost(
    const std::vector<geometry_msgs::msg::Pose>& trajectory,
    const geometry_msgs::msg::Twist& velocity,
    const geometry_msgs::msg::Point& goal,
    const std::vector<ObstacleDetector::Obstacle>& obstacles);

  DWAConfig config_;
};
```

**Cost Function**:
```
Total Cost = α·heading_cost + β·clearance_cost + γ·velocity_cost

where:
  heading_cost = angle between final heading and goal direction
  clearance_cost = 1 / min_obstacle_distance
  velocity_cost = (max_v - v) / max_v
```

#### Phase 3: Local Path Replanning (3-4 hours)

**Goal**: Escape local minima when DWA gets stuck.

**Algorithm**: Use RRT* or A* to find local path around obstacles.

**Implementation**:

```cpp
// include/path_smoothing_control/local_replanner.hpp
#pragma once
#include "obstacle_detector.hpp"
#include <nav_msgs/msg/path.hpp>

class LocalReplanner {
public:
  // Replan local path segment to avoid obstacles
  nav_msgs::msg::Path replanLocalPath(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal,
    const nav_msgs::msg::OccupancyGrid& costmap,
    double max_planning_time = 1.0);  // seconds

private:
  // Simple A* implementation for grid-based planning
  nav_msgs::msg::Path aStarPlanning(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    const nav_msgs::msg::OccupancyGrid& costmap);
  
  // Smooth replanned path using existing B-Spline smoother
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& raw_path);
};
```

#### Phase 4: Integration (2-3 hours)

**Modify Existing Controller Node**:

```cpp
// src/path_smoothing_node.cpp (modifications)
#include "obstacle_detector.hpp"
#include "dwa_planner.hpp"
#include "local_replanner.hpp"

class PathSmoothingNode : public rclcpp::Node {
public:
  PathSmoothingNode() : Node("path_smoothing_node") {
    // Existing subscriptions
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(...);
    
    // NEW: Obstacle detection subscription
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, 
      std::bind(&PathSmoothingNode::scanCallback, this, _1));
    
    // Initialize obstacle avoidance components
    obstacle_detector_ = std::make_unique<ObstacleDetector>();
    dwa_planner_ = std::make_unique<DWAPlanner>();
    local_replanner_ = std::make_unique<LocalReplanner>();
  }

private:
  void controlLoop() {
    // 1. Compute desired velocity from controller (existing)
    auto desired_vel = controller_->computeControl(current_pose_, trajectory_);
    
    // 2. NEW: Check for obstacles
    if (!obstacles_.empty()) {
      // Try DWA first
      auto safe_vel = dwa_planner_->computeSafeVelocity(
        desired_vel, obstacles_, current_pose_, goal_point_);
      
      // If DWA fails (returns zero velocity), trigger replanning
      if (safe_vel.linear.x == 0.0 && safe_vel.angular.z == 0.0) {
        RCLCPP_WARN(get_logger(), "DWA stuck, triggering local replanning");
        auto local_path = local_replanner_->replanLocalPath(
          current_pose_, goal_pose_, costmap_);
        
        if (!local_path.poses.empty()) {
          // Switch to local path temporarily
          trajectory_ = generateTrajectory(local_path);
        }
      } else {
        cmd_vel_pub_->publish(safe_vel);
      }
    } else {
      // No obstacles, use desired velocity
      cmd_vel_pub_->publish(desired_vel);
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacles_ = obstacle_detector_->processLaserScan(*msg);
    costmap_ = obstacle_detector_->generateCostmap(obstacles_);
  }

  std::unique_ptr<ObstacleDetector> obstacle_detector_;
  std::unique_ptr<DWAPlanner> dwa_planner_;
  std::unique_ptr<LocalReplanner> local_replanner_;
  std::vector<ObstacleDetector::Obstacle> obstacles_;
  nav_msgs::msg::OccupancyGrid costmap_;
};
```

### Sensor Requirements

#### Option 1: 2D LiDAR (Recommended)

| Specification | Requirement | Example Sensor |
|--------------|-------------|----------------|
| **Range** | 0.1m - 10m | RPLIDAR A1/A2/A3 |
| **Field of View** | 360° | Hokuyo UST-10LX |
| **Update Rate** | ≥ 10 Hz | SICK TiM551 |
| **Angular Resolution** | ≤ 1° | Slamtec RPLIDAR S1 |
| **ROS 2 Support** | Native driver | rplidar_ros |

**Advantages**:
- ✅ 360° coverage
- ✅ Accurate distance measurements
- ✅ Works in various lighting conditions
- ✅ Mature ROS 2 integration

**ROS 2 Setup**:
```bash
# Install RPLIDAR driver
sudo apt install ros-humble-rplidar-ros

# Launch LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py

# Verify data
ros2 topic echo /scan
```

#### Option 2: Depth Camera

| Specification | Requirement | Example Sensor |
|--------------|-------------|----------------|
| **Range** | 0.3m - 3m | Intel RealSense D435 |
| **Field of View** | ≥ 60° horizontal | Orbbec Astra |
| **Update Rate** | ≥ 30 Hz | Azure Kinect DK |
| **Resolution** | 640x480 minimum | ZED 2 |

**Advantages**:
- ✅ Lower cost than LiDAR
- ✅ Provides RGB + depth
- ✅ Good for indoor environments

**Disadvantages**:
- ❌ Limited range (< 3m typically)
- ❌ Sensitive to lighting
- ❌ Narrower field of view

**ROS 2 Setup**:
```bash
# Install RealSense driver
sudo apt install ros-humble-realsense2-camera

# Launch camera
ros2 launch realsense2_camera rs_launch.py enable_depth:=true

# Convert depth to LaserScan
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node
```

### Testing Strategy

#### Unit Tests

```cpp
// test/test_obstacle_detector.cpp
TEST(ObstacleDetectorTest, DetectsObstaclesFromLaserScan) {
  ObstacleDetector detector;
  
  // Create mock laser scan with obstacle at 1m, 0°
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {1.0, 1.0, 1.0, /* ... */};
  
  auto obstacles = detector.processLaserScan(scan);
  
  EXPECT_GT(obstacles.size(), 0);
  EXPECT_NEAR(obstacles[0].distance, 1.0, 0.1);
}

TEST(DWAPlannerTest, AvoidsObstacles) {
  DWAPlanner planner;
  
  // Desired velocity: forward
  geometry_msgs::msg::Twist desired;
  desired.linear.x = 0.2;
  
  // Obstacle directly ahead
  std::vector<ObstacleDetector::Obstacle> obstacles;
  obstacles.push_back({1.0, 0.0, 1.0, 0.0});
  
  auto safe_vel = planner.computeSafeVelocity(desired, obstacles, ...);
  
  // Should turn to avoid obstacle
  EXPECT_NE(safe_vel.angular.z, 0.0);
}
```

#### Integration Tests

```bash
# Test with mock obstacles in simulation
ros2 launch path_smoothing_control mock_sim_with_obstacles.launch.py

# Test with real LiDAR in controlled environment
ros2 launch path_smoothing_control real_robot_obstacle_test.launch.py
```

### Performance Considerations

| Component | Computational Cost | Optimization |
|-----------|-------------------|--------------|
| **Obstacle Detection** | Low (< 5ms) | Use spatial hashing |
| **DWA Planning** | Medium (10-50ms) | Reduce velocity samples |
| **Local Replanning** | High (50-500ms) | Trigger only when stuck |
| **Total Control Loop** | Target: < 50ms (20 Hz) | Run DWA in separate thread |

### Estimated Development Effort

| Phase | Lines of Code | Time Estimate |
|-------|---------------|---------------|
| Obstacle Detection | ~200 LOC | 2-3 hours |
| DWA Planner | ~300 LOC | 4-6 hours |
| Local Replanner | ~250 LOC | 3-4 hours |
| Integration & Testing | ~150 LOC | 2-3 hours |
| **Total** | **~900 LOC** | **11-16 hours** |

### Alternative Approaches Considered

#### 1. Artificial Potential Fields (APF)

**Pros**: Simple, computationally efficient  
**Cons**: Local minima problems, oscillations  
**Verdict**: ❌ Not recommended for complex environments

#### 2. Model Predictive Control (MPC)

**Pros**: Optimal control, handles constraints  
**Cons**: Computationally expensive, complex tuning  
**Verdict**: ⚠️ Overkill for this application

#### 3. Behavior Trees

**Pros**: Modular, easy to extend  
**Cons**: Requires more infrastructure  
**Verdict**: ⚠️ Good for complex multi-behavior systems

### Next Steps for Implementation

1. **Start with Phase 1**: Implement obstacle detection and verify sensor data
2. **Test in simulation**: Use Gazebo with obstacles before real robot
3. **Implement DWA**: Add reactive obstacle avoidance
4. **Tune parameters**: Adjust cost function weights for desired behavior
5. **Add replanning**: Implement fallback for complex scenarios
6. **Real-world testing**: Deploy to TurtleBot3 with LiDAR

---

## 🤖 Deployment to Real Robot

### Hardware Requirements

**Recommended Platform**: TurtleBot3 Burger or Waffle

**Sensors**:
- **Localization**: Wheel odometry + IMU (minimum), or SLAM with LiDAR for better accuracy
- **Optional**: Camera for visual odometry, GPS for outdoor navigation

**Compute**: Raspberry Pi 4 (4GB+) or equivalent single-board computer

### Software Integration

1. **Replace Mock Simulator**: Instead of launching `mock_sim_node.py`, use the real robot's base driver:
   ```bash
   # For TurtleBot3
   ros2 launch turtlebot3_bringup robot.launch.py
   ```

2. **Launch Control Node Only**:
   ```bash
   ros2 launch path_smoothing_control node_only.launch.py
   ```

3. **Verify Topic Connections**:
   - Ensure `/odom` topic is published by the robot
   - Ensure `/cmd_vel` topic is subscribed by the robot's base controller

### Parameter Tuning for Real Robot

**Critical Parameters to Adjust**:

1. **Velocity Limits**: Reduce for safety during initial testing
   ```yaml
   max_velocity: 0.15  # Start conservative (m/s)
   max_acceleration: 0.3  # Gradual acceleration
   ```

2. **Controller Gains**: Tune based on robot dynamics
   ```yaml
   kp_linear: 0.5      # Reduce if oscillations occur
   kp_angular: 1.5     # Adjust for turning responsiveness
   lookahead_distance: 0.5  # Increase for smoother paths
   ```

3. **Goal Tolerance**: Adjust based on localization accuracy
   ```yaml
   position_tolerance: 0.15  # meters
   heading_tolerance: 0.2    # radians
   ```

### Calibration Procedure

1. **Odometry Calibration**: Verify wheel diameter and wheelbase parameters match your robot
2. **Velocity Calibration**: Test maximum safe velocities in open space
3. **Controller Tuning**: Use Ziegler-Nichols or manual tuning for PID gains
4. **Path Testing**: Start with simple straight-line paths, then progress to curves

### Safety Considerations

- **Emergency Stop**: Implement a safety node that monitors sensor data and can override `cmd_vel`
- **Velocity Ramping**: Enable gradual velocity changes to prevent wheel slip
- **Timeout Detection**: Stop the robot if no new commands are received within a timeout period
- **Collision Avoidance**: Integrate obstacle detection (see Extra Credit section)

### Performance Optimization

- **Control Loop Frequency**: Ensure 20-50 Hz control loop for responsive tracking
- **Path Smoothing Resolution**: Reduce `num_samples` if computation is too slow (default: 200)
- **Localization**: Use AMCL or robot_localization package for improved pose estimation

## AI Tools Used

This project was developed with assistance from AI-powered tools to accelerate development and ensure best practices:

### Tools Utilized

1. **Large Language Models (LLMs)**:
   - Used for algorithm research and implementation guidance
   - Helped design the B-Spline smoothing algorithm with clamped knot vectors
   - Assisted in implementing the trapezoidal velocity profile
   - Provided code review and optimization suggestions

2. **Code Assistance**:
   - Generated boilerplate ROS 2 node structure
   - Helped debug compilation errors and ROS 2 integration issues
   - Suggested test cases for comprehensive coverage
   - Assisted with CMakeLists.txt configuration

3. **Documentation**:
   - Helped structure README with clear explanations
   - Generated algorithm descriptions and design rationale
   - Assisted in creating comprehensive inline code comments

### Development Workflow

The AI tools were used iteratively throughout the development process:
- **Planning Phase**: Researched algorithms and architectural patterns
- **Implementation Phase**: Generated initial code structure, then manually refined
- **Testing Phase**: Designed test cases and debugged failures
- **Documentation Phase**: Created clear explanations of design decisions

All AI-generated code was thoroughly reviewed, tested, and modified to ensure correctness and adherence to project requirements.

## Project Structure

```
path_smoothing_control/
├── config/
│   ├── params.yaml              # Configuration parameters
│   └── path_smoothing.rviz      # RViz configuration
├── include/path_smoothing_control/
│   ├── path_smoother.hpp        # B-Spline smoothing
│   ├── trajectory_generator.hpp # Trajectory generation
│   └── trajectory_controller.hpp# Controllers (Pure Pursuit, Stanley, PID)
├── launch/
│   ├── mock_sim.launch.py       # Mock simulation + RViz
│   ├── node_only.launch.py      # Control node only (for real robot)
│   └── gazebo_world.launch.py  # Gazebo simulation (optional)
├── scripts/
│   ├── mock_sim_node.py         # Lightweight Python simulator
│   └── visualize_trajectory.py  # Matplotlib visualization
├── src/
│   ├── path_smoother.cpp        # Path smoothing implementation
│   ├── trajectory_generator.cpp # Trajectory generation implementation
│   ├── trajectory_controller.cpp# Controller implementations
│   └── path_smoothing_node.cpp  # Main ROS 2 node
├── test/
│   ├── test_path_smoother.cpp   # Path smoother unit tests
│   ├── test_trajectory_generator.cpp # Trajectory generator unit tests
│   └── test_controller.cpp      # Controller unit tests
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Contributing

Contributions are welcome! Please ensure:
- All tests pass (`colcon test`)
- Code follows ROS 2 style guidelines
- New features include unit tests
- Documentation is updated

## License

MIT License

## Acknowledgments

- ROS 2 Humble documentation and tutorials
- "Introduction to Autonomous Mobile Robots" by Siegwart & Nourbakhsh
- Pure Pursuit algorithm from "Implementation of the Pure Pursuit Path Tracking Algorithm" (Coulter, 1992)
- Stanley controller from Stanford's DARPA Grand Challenge entry
