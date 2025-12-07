#include "path_smoothing_control/path_smoother.hpp"
#include <cmath>
#include <iostream>

namespace path_smoothing_control
{

// Helper to create a clamped uniform knot vector
std::vector<double> PathSmoother::generateKnotVector(size_t n, int k)
{
    // n is the number of control points - 1 (indices 0..n) -> size n+1
    // k is the degree
    // m = n + k + 1, number of knots is m+1
    // Number of control points is P = n + 1
    // Knot vector size = P + k + 1
    
    size_t num_control_points = n + 1;
    size_t m = num_control_points + k; 
    std::vector<double> knots;
    knots.resize(m + 1);

    // Clamped uniform options:
    // First k+1 knots are 0
    for (int i = 0; i <= k; ++i) {
        knots[i] = 0.0;
    }

    // Last k+1 knots are 1
    for (int i = m - k; i <= (int)m; ++i) {
        knots[i] = 1.0;
    }

    // Interior knots are equally spaced
    // Interior knots are equally spaced

    
    // Simpler loop for internal knots
    // Count of defined knots so far is k+1 (indices 0 to k).
    // Count of knots at end is k+1.
    // Total knots = num_control_points + k + 1
    // Num middle knots = Total - 2(k+1) = num_control_points - k - 1
    
    double demonimator = (double)(num_control_points - k);
    
    for (size_t i = k + 1; i < num_control_points; ++i) {
        knots[i] = (double)(i - k) / demonimator;
    }
    
    return knots;
}

double PathSmoother::basisFunction(int i, int k, double t, const std::vector<double>& knots)
{
    if (k == 0) {
        if (t >= knots[i] && t < knots[i+1]) return 1.0;
        // Special case for the very last point t=1.0
        if (std::abs(t - 1.0) < 1e-9 && std::abs(knots[i+1] - 1.0) < 1e-9) return 1.0;
        return 0.0;
    }

    double term1 = 0.0;
    double denom1 = knots[i+k] - knots[i];
    if (denom1 > 0.0) {
        term1 = ((t - knots[i]) / denom1) * basisFunction(i, k-1, t, knots);
    }

    double term2 = 0.0;
    double denom2 = knots[i+k+1] - knots[i+1];
    if (denom2 > 0.0) {
        term2 = ((knots[i+k+1] - t) / denom2) * basisFunction(i+1, k-1, t, knots);
    }

    return term1 + term2;
}

std::vector<Point2D> PathSmoother::bSplineSmoothing(
    const std::vector<Point2D>& raw_waypoints, 
    size_t num_samples, 
    int degree)
{
    std::vector<Point2D> smooth_path;
    if (raw_waypoints.size() < (size_t)(degree + 1)) {
        return raw_waypoints; // Not enough points for degree
    }

    // n is index of last control point
    size_t n = raw_waypoints.size() - 1;
    auto knots = generateKnotVector(n, degree);

    smooth_path.reserve(num_samples);
    double dt = 1.0 / (double)(num_samples - 1);

    for (size_t j = 0; j < num_samples; ++j) {
        double t = j * dt;
        if (t > 1.0) t = 1.0;

        double x = 0.0;
        double y = 0.0;

        // Evaluate B-Spline at t
        // P(t) = Sum_{i=0 to n} N_{i,k}(t) * P_i
        for (size_t i = 0; i < raw_waypoints.size(); ++i) {
            double basis = basisFunction(i, degree, t, knots);
            x += basis * raw_waypoints[i].x;
            y += basis * raw_waypoints[i].y;
        }
        smooth_path.push_back({x, y});
    }

    return smooth_path;
}

std::vector<Point2D> PathSmoother::smoothPath(
    const std::vector<Point2D>& raw_waypoints,
    size_t num_points,
    SmoothingType type)
{
    if (raw_waypoints.size() < 2) return raw_waypoints;

    switch (type) {
        case SmoothingType::BSPLINE:
            // Use degree 3 (cubic) B-Spline by default
            // If fewer points than 4, reduce degree
            {
                int degree = 3;
                if (raw_waypoints.size() <= 3) degree = raw_waypoints.size() - 1;
                return bSplineSmoothing(raw_waypoints, num_points, degree);
            }
        case SmoothingType::BEZIER:
            // Not implemented, fallback
            return raw_waypoints;
        default:
            return raw_waypoints;
    }
}

} // namespace path_smoothing_control
