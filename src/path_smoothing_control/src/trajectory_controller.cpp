#include "path_smoothing_control/trajectory_controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

namespace path_smoothing_control
{

double TrajectoryController::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

TrajectoryController::TrajectoryController(
    double lookahead_dist,
    double max_lin_vel,
    double max_ang_vel)
    : lookahead_distance_(lookahead_dist),
      max_linear_velocity_(max_lin_vel),
      max_angular_velocity_(max_ang_vel),
      kp_linear_(1.0),
      kp_angular_(2.0),
      ki_(0.0),
      kd_(0.1)
{}

void TrajectoryController::setTrajectory(const std::vector<TrajectoryPoint>& trajectory)
{
    (void)trajectory;
}

void TrajectoryController::setGains(double kp_linear, double kp_angular, double ki, double kd)
{
    kp_linear_ = kp_linear;
    kp_angular_ = kp_angular;
    ki_ = ki;
    kd_ = kd;
}

void TrajectoryController::setLookaheadDistance(double dist)
{
    lookahead_distance_ = std::max(0.1, dist);
}

size_t TrajectoryController::findClosestPoint(
    const RobotPose& pose,
    const std::vector<TrajectoryPoint>& trajectory)
{
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = last_closest_index_;

    for (size_t i = closest_idx; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - pose.x;
        double dy = trajectory[i].y - pose.y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        } else if (i > closest_idx + 10) {
            break;
        }
    }

    last_closest_index_ = closest_idx;
    return closest_idx;
}

TrajectoryPoint TrajectoryController::findLookaheadPoint(
    const RobotPose& pose,
    const std::vector<TrajectoryPoint>& trajectory,
    size_t start_idx)
{
    for (size_t i = start_idx; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - pose.x;
        double dy = trajectory[i].y - pose.y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_distance_) {
            return trajectory[i];
        }
    }
    return trajectory.back();
}

double TrajectoryController::computeCrossTrackError(
    const RobotPose& pose,
    const TrajectoryPoint& target)
{
    double dx = target.x - pose.x;
    double dy = target.y - pose.y;
    double target_vector_heading = std::atan2(dy, dx);
    double heading_diff = normalizeAngle(target.heading - target_vector_heading);
    double distance = std::hypot(dx, dy);
    return distance * std::sin(heading_diff);
}

bool TrajectoryController::isGoalReached(
    const RobotPose& current_pose,
    const TrajectoryPoint& goal,
    double position_tolerance,
    double heading_tolerance)
{
    double dx = goal.x - current_pose.x;
    double dy = goal.y - current_pose.y;
    double distance = std::hypot(dx, dy);
    double heading_error = std::abs(normalizeAngle(goal.heading - current_pose.theta));
    return (distance < position_tolerance) && (heading_error < heading_tolerance);
}

geometry_msgs::msg::Twist TrajectoryController::purePursuitControl(
    const RobotPose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory,
    double current_time)
{
    (void)current_time;
    geometry_msgs::msg::Twist cmd_vel;
    size_t closest_idx = findClosestPoint(current_pose, trajectory);
    TrajectoryPoint lookahead = findLookaheadPoint(current_pose, trajectory, closest_idx);

    double dx = lookahead.x - current_pose.x;
    double dy = lookahead.y - current_pose.y;

    double cos_theta = std::cos(current_pose.theta);
    double sin_theta = std::sin(current_pose.theta);
    double dx_robot = dx * cos_theta + dy * sin_theta;
    double dy_robot = -dx * sin_theta + dy * cos_theta;

    double distance_sq = dx_robot * dx_robot + dy_robot * dy_robot;
    double curvature = 0.0;
    if (distance_sq > 1e-6) {
        curvature = 2.0 * dy_robot / distance_sq;
    }

    double target_velocity = trajectory[closest_idx].velocity;
    cmd_vel.linear.x = std::clamp(target_velocity, 0.0, max_linear_velocity_);
    cmd_vel.angular.z = curvature * cmd_vel.linear.x;

    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
                                     -max_angular_velocity_,
                                     max_angular_velocity_);

    tracking_error_ = std::hypot(dx, dy);
    return cmd_vel;
}

geometry_msgs::msg::Twist TrajectoryController::stanleyControl(
    const RobotPose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory,
    double current_time)
{
    (void)current_time;
    geometry_msgs::msg::Twist cmd_vel;
    size_t closest_idx = findClosestPoint(current_pose, trajectory);
    const TrajectoryPoint& target = trajectory[closest_idx];

    double heading_error = normalizeAngle(target.heading - current_pose.theta);
    double cte = computeCrossTrackError(current_pose, target);

    double velocity = std::clamp(target.velocity, 0.0, max_linear_velocity_);
    double cte_term = std::atan2(kp_angular_ * cte, velocity + 0.1);

    cmd_vel.linear.x = velocity;
    cmd_vel.angular.z = heading_error + cte_term;

    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
                                     -max_angular_velocity_,
                                     max_angular_velocity_);

    tracking_error_ = std::abs(cte);
    return cmd_vel;
}

geometry_msgs::msg::Twist TrajectoryController::pidControl(
    const RobotPose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory,
    double current_time)
{
    (void)current_time;
    geometry_msgs::msg::Twist cmd_vel;
    size_t closest_idx = findClosestPoint(current_pose, trajectory);
    const TrajectoryPoint& target = trajectory[closest_idx];

    double dx = target.x - current_pose.x;
    double dy = target.y - current_pose.y;
    double distance_error = std::hypot(dx, dy);

    double desired_heading = std::atan2(dy, dx);
    double heading_error = normalizeAngle(desired_heading - current_pose.theta);

    const double dt = 0.05;

    integral_error_ += distance_error * dt;
    double derivative_error = (distance_error - previous_error_) / dt;

    double linear_control = kp_linear_ * distance_error +
                            ki_ * integral_error_ +
                            kd_ * derivative_error;

    cmd_vel.linear.x = std::clamp(linear_control, 0.0, max_linear_velocity_);
    cmd_vel.angular.z = kp_angular_ * heading_error;
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
                                     -max_angular_velocity_,
                                     max_angular_velocity_);

    tracking_error_ = distance_error;
    previous_error_ = distance_error;
    return cmd_vel;
}

geometry_msgs::msg::Twist TrajectoryController::computeControl(
    const RobotPose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory,
    double current_time,
    ControllerType type)
{
    if (trajectory.empty()) {
        return geometry_msgs::msg::Twist();
    }

    if (current_time < 0.05) last_closest_index_ = 0;

    switch (type) {
        case ControllerType::PURE_PURSUIT:
            return purePursuitControl(current_pose, trajectory, current_time);
        case ControllerType::STANLEY:
            return stanleyControl(current_pose, trajectory, current_time);
        case ControllerType::PID:
            return pidControl(current_pose, trajectory, current_time);
        default:
            return purePursuitControl(current_pose, trajectory, current_time);
    }
}

} // namespace path_smoothing_control
