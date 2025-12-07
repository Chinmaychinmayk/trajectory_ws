#include "path_smoothing_control/trajectory_generator.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace path_smoothing_control
{
TrajectoryGenerator::TrajectoryGenerator(double max_v, double max_a)
    : max_v_(max_v), max_a_(max_a) {}

std::vector<double> TrajectoryGenerator::calculateHeadings(const std::vector<Point2D>& path)
{
    std::vector<double> headings;
    if (path.size() < 2) return {0.0};

    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        headings.push_back(std::atan2(dy, dx));
    }
    // Set the last heading to be the same as the second-to-last
    headings.push_back(headings.back()); 
    return headings;
}

std::vector<double> TrajectoryGenerator::calculateVelocityProfile(
    const std::vector<double>& distances, 
    VelocityProfile profile)
{
    std::vector<double> velocities;
    if (profile != VelocityProfile::TRAPEZOIDAL) {
        // Fallback or simple constant velocity profile
        velocities.resize(distances.size(), max_v_ * 0.5);
        velocities.front() = 0.0;
        velocities.back() = 0.0;
        return velocities;
    }
    
    // Trapezoidal Profile Implementation
    double L_total = distances.back();
    double d_a = max_v_ * max_v_ / (2.0 * max_a_); // Distance to reach max_v
    velocities.reserve(distances.size());

    for (double d : distances) {
        double v = 0.0;
        if (d <= d_a) {
            v = std::sqrt(2.0 * max_a_ * d);
        } else if (d > d_a && d <= (L_total - d_a)) {
            v = max_v_;
        } else {
            double d_remain = L_total - d;
            v = std::sqrt(std::max(0.0, 2.0 * max_a_ * d_remain));
        }
        velocities.push_back(std::min(v, max_v_));
    }
    
    // Enforce v=0 at start and end
    if (!velocities.empty()) {
        velocities.front() = 0.0;
        velocities.back() = 0.0;
    }
    
    return velocities;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(
    const std::vector<Point2D>& path, 
    VelocityProfile profile)
{
    std::vector<TrajectoryPoint> trajectory;
    if (path.size() < 2) return trajectory;

    // 1. Calculate cumulative arc length
    std::vector<double> distances;
    distances.push_back(0.0);
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        double dl = std::hypot(dx, dy);
        distances.push_back(distances.back() + dl);
    }

    // 2. Calculate linear velocity profile 
    std::vector<double> velocities = calculateVelocityProfile(distances, profile);

    // 3. Calculate timestamps (integration of inverse velocity)
    std::vector<double> timestamps;
    timestamps.push_back(0.0);
    double current_time = 0.0;

    for (size_t i = 0; i < distances.size() - 1; ++i) {
        double dl = distances[i+1] - distances[i];
        double v_avg = (velocities[i+1] + velocities[i]) / 2.0;
        
        // Safety: Avoid division by zero/near-zero
        if (v_avg < 0.01) v_avg = 0.01;
        
        current_time += dl / v_avg;
        timestamps.push_back(current_time);
    }
    total_time_ = current_time;

    // 4. Calculate headings
    std::vector<double> headings = calculateHeadings(path);

    // 5. Combine into TrajectoryPoint structure
    for (size_t i = 0; i < path.size(); ++i) {
        trajectory.push_back({
            path[i].x,
            path[i].y,
            headings[i],
            velocities[i],
            timestamps[i]
        });
    }

    return trajectory;
}
} // namespace path_smoothing_control

