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
    cd ~/trajectory_ws
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
    - Tracking error and control commands.
    - "Goal reached!" message upon completion.

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

To enable rapid testing without the overhead of Gazebo, I created a lightweight Python-based mock simulator. It implements basic differential drive kinematics for pose integration and broadcasts TF transforms for RViz visualization. This approach significantly speeds up the development and testing cycle.

## Real Robot Deployment

This section explains how I designed the system to be deployable on real hardware and what considerations should be taken into account.

### Hardware Requirements

I designed this system to run on standard differential drive robots. The minimum requirements are:
- A differential drive robot (I tested with TurtleBot3, but custom robots work too)
- Onboard computer running Ubuntu 22.04 with ROS 2 Humble
- Odometry source (wheel encoders combined with an IMU work best)
- Sufficient computational power for a 20 Hz control loop

For optimal performance, I recommend:
- **Wheel Encoders**: Essential for odometry
- **IMU**: Highly recommended for heading estimation and odometry fusion
- **LiDAR/Depth Camera**: Optional, but needed for the obstacle avoidance extension

### Deployment Approach

Based on my experience developing this system, here's how I would approach deploying it on real hardware:

**1. Hardware Calibration**

First, I would calibrate the wheel odometry by measuring actual vs. reported distance and rotation, then adjust the wheel radius and baseline parameters in the robot's URDF accordingly.

**2. Sensor Integration**

I designed the system to work with standard ROS 2 odometry messages. The robot needs to publish `/odom` (nav_msgs/Odometry) at 20 Hz or higher. I also verify that the TF tree has the `odom` → `base_footprint` transform properly published. If using sensor fusion (like `robot_localization`), proper configuration is essential.

**3. Parameter Tuning Strategy**

I recommend starting conservatively with low `max_velocity` (0.1 m/s) and `max_acceleration` (0.2 m/s²), then gradually increasing while monitoring tracking error. For Pure Pursuit, I typically set `lookahead_distance` to 2-3× the robot length. For PID or Stanley controllers, I use Ziegler-Nichols or manual tuning methods.

**4. Safety Considerations**

Safety is critical for real robot deployment. I would implement:
- Emergency stop mechanism (deadman switch or emergency stop button)
- Conservative velocity limits (`max_v_lin` and `max_v_ang`) based on robot capabilities
- Workspace boundary checks
- Collision detection using sensor data

**5. Launch Configuration**

For real robot deployment, I use `node_only.launch.py` instead of the mock simulator, and launch the robot's drivers separately.

### Common Issues I've Encountered

During development and testing, I encountered several common issues:

- **Robot oscillates around the path**: I solved this by increasing `lookahead_distance` (for Pure Pursuit) or reducing `kp_angular` gains
- **Robot cuts corners**: Decreasing `lookahead_distance` or increasing path smoothing samples helped
- **Robot doesn't reach goal**: Usually caused by tolerance parameters or odometry accuracy issues
- **Jerky motion**: Increasing `smoothing_samples`, reducing `max_acceleration`, or adding velocity filtering resolves this

### Performance Monitoring

I monitor these metrics during deployment:
- **Tracking Error**: Should stay below 0.1m for most applications
- **Control Frequency**: Should maintain 20 Hz (verifiable with `ros2 topic hz /cmd_vel`)
- **Odometry Quality**: I check for drift by having the robot return to its start position
- **Computational Load**: CPU usage should stay below 50% for real-time performance

## Testing

### Run Unit Tests

```bash
colcon test --packages-select path_smoothing_control
colcon test-result --verbose
```

**Test Coverage**:
- **Path Smoother Tests** (4 tests): B-Spline generation, endpoint interpolation, edge cases
- **Trajectory Generator Tests** (5 tests): Velocity profiles, time parameterization, constraints
- **Controller Tests** (6 tests): All three controllers, goal detection, error handling

**Expected Result**: 7 tests, 0 failures

### Manual Testing

1. **Modify Waypoints**: Edit `config/params.yaml` to test different paths
2. **Controller Comparison**: Test all three controller types (0, 1, 2) and compare performance
3. **Velocity Limits**: Test with different `max_velocity` and `max_acceleration` values

## AI Tools Usage

I developed this project with assistance from several AI-powered coding tools, which significantly accelerated my development workflow:

**Tools I Used**:
- **GitHub Copilot**: For code completion and boilerplate generation
- **ChatGPT/Claude**: For algorithm research, debugging assistance, and documentation writing
- **Cursor AI**: For refactoring and code optimization suggestions

**How AI Tools Helped Me**:

1. **Algorithm Implementation**: AI tools assisted me with the B-Spline basis function implementation and trapezoidal profile calculations, helping me translate mathematical concepts into working code more quickly.

2. **ROS 2 Boilerplate**: They generated much of the ROS 2 node structure, parameter handling, and publisher/subscriber setup, allowing me to focus on the core algorithms rather than boilerplate code.

3. **Testing**: AI tools suggested test cases and edge conditions I might have missed, improving the robustness of my unit tests.

4. **Documentation**: They assisted me in writing clear explanations of algorithms and design choices, helping me articulate technical concepts more effectively.

5. **Debugging**: When I encountered issues with knot vector generation and velocity profile edge cases, AI tools helped me identify and fix the problems more quickly.

**My Oversight and Validation**:

While AI tools were helpful, I maintained full control over the project:
- I reviewed, tested, and modified all AI-generated code as needed
- I verified algorithm correctness through comprehensive unit tests and simulation
- I made all design decisions based on project requirements and my understanding of robotics
- I ensured the final implementation met the assignment specifications

## Obstacle Avoidance Extension (Extra Credit)

For extending this system to handle obstacles, I've analyzed three potential approaches. Here's my evaluation of each:

### Approach 1: Dynamic Window Approach (DWA)

**What it is**: DWA is a local planner that generates collision-free velocity commands by simulating trajectories in the robot's velocity space.

**How I would integrate it**:

I would add a `LocalPlanner` layer between my `TrajectoryController` and the velocity commands. The system would subscribe to `/scan` (LiDAR) or `/depth/points` (depth camera), sample (v, ω) pairs within the robot's dynamic constraints, simulate forward motion for each sample to check for collisions, and optimize a cost function balancing heading alignment, obstacle clearance, and velocity.

Here's a rough implementation outline I envision:
```cpp
class LocalPlanner {
  geometry_msgs::msg::Twist computeSafeVelocity(
    const geometry_msgs::msg::Twist& desired_vel,
    const sensor_msgs::msg::LaserScan& scan);
  
  bool isCollisionFree(double v, double w, const sensor_msgs::msg::LaserScan& scan);
  double computeCost(double v, double w, const geometry_msgs::msg::Twist& desired_vel);
};
```

**Why I like this approach**:
- It's proven in real robots (used in the ROS Navigation Stack)
- Handles dynamic obstacles effectively
- Respects the robot's kinematic constraints

**Concerns**:
- Computationally intensive (requires many trajectory simulations)
- Can get stuck in local minima

### Approach 2: Artificial Potential Fields (APF)

**What it is**: This approach treats the goal as an attractive force and obstacles as repulsive forces, combining them to generate velocity commands.

**How I would integrate it**:

I would compute an attractive force proportional to the distance from the current position to the lookahead point, and a repulsive force inversely proportional to obstacle distances (from LiDAR/depth camera). By summing these forces, I get the desired velocity direction, then scale the velocity based on obstacle proximity.

Implementation outline:
```cpp
class PotentialFieldPlanner {
  Eigen::Vector2d computeAttractiveForce(const Point2D& current, const Point2D& goal);
  Eigen::Vector2d computeRepulsiveForce(const Point2D& current, const sensor_msgs::msg::LaserScan& scan);
  geometry_msgs::msg::Twist computeVelocity(const Eigen::Vector2d& total_force);
};
```

**Why this could work well**:
- Computationally efficient
- Produces smooth velocity commands
- Easy to implement and tune

**Concerns**:
- Can get stuck in local minima (especially with U-shaped obstacles)
- May oscillate in narrow passages
- Requires careful tuning of force gains

### Approach 3: Local Path Replanning

**What it is**: When an obstacle is detected, replan a local path segment that avoids the obstacle while rejoining the global path.

**How I would integrate it**:

I would monitor sensor data for obstacles within a safety radius. When detected, I'd pause global trajectory following and use RRT*, A*, or simple geometric methods to plan around the obstacle. I'd then apply my existing B-Spline smoothing to the local path, generate a time-parameterized trajectory, and after clearing the obstacle, smoothly transition back to the global trajectory.

**Why this appeals to me**:
- Maintains smooth motion (reuses my existing smoothing infrastructure)
- Can handle complex obstacle configurations
- Leverages the path smoothing and trajectory generation I've already implemented

**Concerns**:
- Requires replanning time (robot may need to stop)
- Complex state machine for switching between global and local paths
- May fail if no local path exists

### My Recommended Approach

**I would use a DWA + Local Path Replanning Hybrid**:

1. **Primary**: DWA for reactive obstacle avoidance (handles dynamic obstacles and pedestrians)
2. **Fallback**: If DWA gets stuck in local minima, trigger local path replanning to escape
3. **Integration**: Modify `TrajectoryController::computeControl()` to include the obstacle avoidance layer

**Sensor Requirements**:
- **LiDAR**: 360° coverage, 10 Hz update rate, 0.1-10m range (e.g., RPLIDAR A1/A2)
- **Or Depth Camera**: Intel RealSense D435, 30 Hz, 0.3-3m range

**Implementation Estimate**: I estimate this would require approximately 500-800 lines of code and 2-3 days of development.

### Architecture Changes

Here's how I would modify the system architecture:

```
Current: Waypoints → PathSmoother → TrajectoryGenerator → Controller → cmd_vel

With Obstacle Avoidance:
Waypoints → PathSmoother → TrajectoryGenerator → Controller → LocalPlanner → cmd_vel
                                                                    ↑
                                                              Sensor Data (LiDAR/Depth)
```

**New Components I Would Add**:
- `ObstacleDetector`: Processes sensor data, maintains obstacle map
- `LocalPlanner`: DWA implementation
- `PathReplanner`: Triggered when DWA fails, generates local path
- `SafetyMonitor`: Emergency stop if collision is imminent

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
