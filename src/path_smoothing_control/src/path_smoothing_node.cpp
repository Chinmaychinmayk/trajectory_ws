#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "path_smoothing_control/path_smoother.hpp"
#include "path_smoothing_control/trajectory_generator.hpp"
#include "path_smoothing_control/trajectory_controller.hpp"
#include <vector>
#include <string>

using namespace std::chrono_literals;
using namespace path_smoothing_control;

class PathSmoothingNode : public rclcpp::Node
{
public:
  PathSmoothingNode() : Node("path_smoothing_node")
  {
    // 1. Declare and Get Parameters
    this->declare_parameter("waypoints_x", std::vector<double>{});
    this->declare_parameter("waypoints_y", std::vector<double>{});
    this->declare_parameter("smoothing_samples", 200);
    this->declare_parameter("max_velocity", 0.2);
    this->declare_parameter("max_acceleration", 0.5);
    this->declare_parameter("lookahead_distance", 0.3);
    this->declare_parameter("control_frequency", 20.0);
    this->declare_parameter("controller_type", 0);
    this->declare_parameter("kp_linear", 1.0);
    this->declare_parameter("kp_angular", 2.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.1);
    this->declare_parameter("max_v_lin", 0.22);
    this->declare_parameter("max_v_ang", 2.84);

    // 2. Initialize Components
    double max_v = this->get_parameter("max_velocity").as_double();
    double max_a = this->get_parameter("max_acceleration").as_double();
    double Ld = this->get_parameter("lookahead_distance").as_double();
    double max_v_lin = this->get_parameter("max_v_lin").as_double();
    double max_v_ang = this->get_parameter("max_v_ang").as_double();

    path_smoother_ = std::make_shared<PathSmoother>();
    trajectory_generator_ = std::make_shared<TrajectoryGenerator>(max_v, max_a);
    trajectory_controller_ = std::make_shared<TrajectoryController>(Ld, max_v_lin, max_v_ang);
    
    // Set controller gains
    trajectory_controller_->setGains(
        this->get_parameter("kp_linear").as_double(),
        this->get_parameter("kp_angular").as_double(),
        this->get_parameter("ki").as_double(),
        this->get_parameter("kd").as_double()
    );

    // 3. Publishers and Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_path", 10);
    trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/trajectory_marker", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PathSmoothingNode::odomCallback, this, std::placeholders::_1));

    // 4. Generate Path and Trajectory
    loadWaypoints();
    generateSmoothTrajectory();
    publishPaths();

    // 5. Control timer
    double control_freq = this->get_parameter("control_frequency").as_double();
    auto period = std::chrono::duration<double>(1.0 / control_freq);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PathSmoothingNode::controlLoop, this));

    trajectory_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Path Smoothing Node initialized");
    RCLCPP_INFO(this->get_logger(), "Trajectory points: %zu", trajectory_.size());
    RCLCPP_INFO(this->get_logger(), "Total trajectory time: %.2f seconds", trajectory_generator_->getTotalTime());
  }

private:
  void loadWaypoints()
  {
    auto x_coords = this->get_parameter("waypoints_x").as_double_array();
    auto y_coords = this->get_parameter("waypoints_y").as_double_array();

    if (x_coords.size() != y_coords.size() || x_coords.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Waypoint coordinate size mismatch or empty.");
      return;
    }

    waypoints_.clear();
    for (size_t i = 0; i < x_coords.size(); ++i) {
      waypoints_.push_back({x_coords[i], y_coords[i]});
    }
  }

  void generateSmoothTrajectory()
  {
    if (waypoints_.size() < 2) return;

    // Smooth the path
    int num_samples = this->get_parameter("smoothing_samples").as_int();
    // Assuming BSPLINE is the default implemented type
    smooth_path_ = path_smoother_->smoothPath(waypoints_, num_samples, SmoothingType::BSPLINE);

    // Generate time-parameterized trajectory
    // Assuming TRAPEZOIDAL is the default implemented profile
    trajectory_ = trajectory_generator_->generateTrajectory(smooth_path_, VelocityProfile::TRAPEZOIDAL);
    
    // Set the trajectory for the controller
    trajectory_controller_->setTrajectory(trajectory_);
  }

  void publishPaths()
  {
    auto now = this->now();
    nav_msgs::msg::Path waypoint_path;
    nav_msgs::msg::Path smooth_path_msg;
    
    // Setup headers
    waypoint_path.header.stamp = now;
    waypoint_path.header.frame_id = "odom";
    smooth_path_msg.header = waypoint_path.header;

    // Publish original waypoints
    for (const auto& wp : waypoints_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = waypoint_path.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      waypoint_path.poses.push_back(pose);
    }
    path_pub_->publish(waypoint_path);

    // Publish smooth path
    for (const auto& pt : smooth_path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = smooth_path_msg.header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      smooth_path_msg.poses.push_back(pose);
    }
    smooth_path_pub_->publish(smooth_path_msg);

    // Publish trajectory markers
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now;
    marker.header.frame_id = "odom";
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& tp : trajectory_) {
      geometry_msgs::msg::Point p;
      p.x = tp.x;
      p.y = tp.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    trajectory_marker_pub_->publish(marker);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose_.theta = yaw;

    odom_received_ = true;
  }

  void controlLoop()
  {
    if (!odom_received_ || trajectory_.empty()) return;

    double elapsed = (this->now() - trajectory_start_time_).seconds();

    // Check if goal reached (using final trajectory point as goal)
    if (trajectory_controller_->isGoalReached(
        current_pose_, trajectory_.back(), 0.15, 0.2))
    {
      if (!goal_reached_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached! Total time: %.2f", elapsed);
        goal_reached_ = true;
        
        geometry_msgs::msg::Twist stop_cmd; // Send zero command
        cmd_vel_pub_->publish(stop_cmd);
      }
      return;
    }
    
    // Get controller type
    int controller_type_int = this->get_parameter("controller_type").as_int();
    ControllerType ctrl_type = static_cast<ControllerType>(controller_type_int);

    // Compute control command
    auto cmd_vel = trajectory_controller_->computeControl(
      current_pose_, trajectory_, elapsed, ctrl_type);

    // Publish command
    cmd_vel_pub_->publish(cmd_vel);

    // Log tracking error periodically
    if (++log_counter_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Time: %.1fs, Error: %.3fm, Cmd: linear=%.2f, angular=%.2f",
                  elapsed,
                  trajectory_controller_->getTrackingError(),
                  cmd_vel.linear.x, cmd_vel.angular.z);
    }
  }

  // Components and Data
  std::shared_ptr<PathSmoother> path_smoother_;
  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  std::shared_ptr<TrajectoryController> trajectory_controller_;

  std::vector<Point2D> waypoints_;
  std::vector<Point2D> smooth_path_;
  std::vector<TrajectoryPoint> trajectory_;
  RobotPose current_pose_;

  // ROS2 interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  bool odom_received_ = false;
  bool goal_reached_ = false;
  rclcpp::Time trajectory_start_time_;
  int log_counter_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSmoothingNode>());
  rclcpp::shutdown();
  return 0;
}
