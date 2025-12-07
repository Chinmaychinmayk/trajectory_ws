#ifndef TRAJECTORY_CONTROLLER_HPP
#define TRAJECTORY_CONTROLLER_HPP

#include <vector>
#include <limits>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_generator.hpp" // For TrajectoryPoint

namespace path_smoothing_control
{
/**
 * @brief Robot's current pose from odometry.
 */
struct RobotPose {
    double x, y, theta;
    RobotPose(double x_in=0.0, double y_in=0.0, double theta_in=0.0) 
        : x(x_in), y(y_in), theta(theta_in) {}
};

/**
 * @brief Available controller types.
 */
enum class ControllerType {
    PURE_PURSUIT = 0,
    STANLEY = 1,
    PID = 2
};

/**
 * @brief Class to implement various trajectory tracking controllers.
 */
class TrajectoryController
{
public:
    TrajectoryController(double lookahead_dist, double max_lin_vel, double max_ang_vel);
    ~TrajectoryController() = default;

    void setTrajectory(const std::vector<TrajectoryPoint>& trajectory);
    void setGains(double kp_linear, double kp_angular, double ki, double kd);
    void setLookaheadDistance(double dist);

    geometry_msgs::msg::Twist computeControl(
        const RobotPose& current_pose,
        const std::vector<TrajectoryPoint>& trajectory,
        double current_time,
        ControllerType type
    );

    bool isGoalReached(
        const RobotPose& current_pose,
        const TrajectoryPoint& goal,
        double position_tolerance,
        double heading_tolerance
    );

    double getTrackingError() const { return tracking_error_; }
    TrajectoryPoint getLookaheadPoint() const { return current_lookahead_; }
    size_t getClosestIndex() const { return last_closest_index_; }
    
private:
    // Parameters
    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    
    // Gains
    double kp_linear_;
    double kp_angular_;
    double ki_;
    double kd_;

    // Tracking State
    double tracking_error_ = 0.0;
    double integral_error_ = 0.0;
    double previous_error_ = 0.0;
    size_t last_closest_index_ = 0; // Optimization hint
    TrajectoryPoint current_lookahead_;

    // Control Implementations
    geometry_msgs::msg::Twist purePursuitControl(
        const RobotPose& current_pose, 
        const std::vector<TrajectoryPoint>& trajectory, 
        double current_time
    );

    geometry_msgs::msg::Twist stanleyControl(
        const RobotPose& current_pose, 
        const std::vector<TrajectoryPoint>& trajectory, 
        double current_time
    );

    geometry_msgs::msg::Twist pidControl(
        const RobotPose& current_pose, 
        const std::vector<TrajectoryPoint>& trajectory, 
        double current_time
    );
    
    // Helpers
    size_t findClosestPoint(const RobotPose& pose, const std::vector<TrajectoryPoint>& trajectory);
    TrajectoryPoint findLookaheadPoint(
        const RobotPose& pose, 
        const std::vector<TrajectoryPoint>& trajectory, 
        size_t start_idx
    );
    double normalizeAngle(double angle);
    double computeCrossTrackError(const RobotPose& pose, const TrajectoryPoint& target);
};
} // namespace path_smoothing_control

#endif // TRAJECTORY_CONTROLLER_HPP
