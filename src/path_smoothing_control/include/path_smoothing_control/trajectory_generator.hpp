#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <vector>
#include <cmath>
#include "path_smoother.hpp" // For Point2D

namespace path_smoothing_control
{
/**
 * @brief Trajectory point including kinematic state and time.
 */
struct TrajectoryPoint {
    double x, y;
    double heading;         // Radian
    double velocity;        // m/s
    double time;            // seconds
};

/**
 * @brief Available velocity profile types.
 */
enum class VelocityProfile {
    TRAPEZOIDAL = 0,
    CONSTANT = 1,
    S_CURVE = 2
};

/**
 * @brief Class to generate a time-parameterized trajectory.
 */
class TrajectoryGenerator
{
public:
    TrajectoryGenerator(double max_v, double max_a);
    ~TrajectoryGenerator() = default;

    std::vector<TrajectoryPoint> generateTrajectory(
        const std::vector<Point2D>& path, 
        VelocityProfile profile = VelocityProfile::TRAPEZOIDAL
    );

    double getTotalTime() const { return total_time_; }

private:
    double max_v_;
    double max_a_;
    double total_time_ = 0.0;

    std::vector<double> calculateHeadings(const std::vector<Point2D>& path);
    std::vector<double> calculateVelocityProfile(
        const std::vector<double>& distances, 
        VelocityProfile profile
    );
};
} // namespace path_smoothing_control

#endif // TRAJECTORY_GENERATOR_HPP
