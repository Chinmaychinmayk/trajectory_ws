#include "path_smoothing_control/path_smoother.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace path_smoothing_control
{

// Implementation of Cox-de Boor recursion formula
double PathSmoother::basisFunction(int i, int k, double t, const std::vector<double>& knots)
{
    if (k == 1) {
        return (t >= knots[i] && t < knots[i+1]) ? 1.0 : 0.0;
    }

    double term1 = 0.0;
    if (knots[i+k-1] != knots[i]) {
        term1 = (t - knots[i]) / (knots[i+k-1] - knots[i]) * basisFunction(i, k - 1, t, knots);
    }

    double term2 = 0.0;
    if (knots[i+k] != knots[i+1]) {
        term2 = (knots[i+k] - t) / (knots[i+k] - knots[i+1]) * basisFunction(i + 1, k - 1, t, knots);
    }

    return term1 + term2;
}

// Generate uniform clamped knot vector
std::vector<double> PathSmoother::generateKnotVector(size_t n, int k)
{
    size_t m = n + k;
    std::vector<double> knots(m + 1);

    for (int i = 0; i <= k; ++i) {
        knots[i] = 0.0;
        knots[m - i] = 1.0;
    }

    for (size_t i = k + 1; i <= m - k - 1; ++i) {
        knots[i] = static_cast<double>(i - k) / (static_cast<double>(n - k));
    }

    return knots;
}

std::vector<Point2D> PathSmoother::bSplineSmoothing(
    const std::vector<Point2D>& raw_waypoints,
    size_t num_points,
    int degree)
{
    std::vector<Point2D> smoothed_path;
    const size_t n = raw_waypoints.size();
    const int k = degree;

    if (n < (size_t)degree + 1) return smoothed_path;

    std::vector<double> knots = generateKnotVector(n, k);

    for (size_t j = 0; j < num_points; ++j) {
        double t = static_cast<double>(j) / static_cast<double>(num_points - 1);
        if (j == num_points - 1) t = 1.0;

        double p_x = 0.0;
        double p_y = 0.0;

        for (size_t i = 0; i < n; ++i) {
            double N_i_k = basisFunction(i, k + 1, t, knots);
            p_x += N_i_k * raw_waypoints[i].x;
            p_y += N_i_k * raw_waypoints[i].y;
        }

        smoothed_path.push_back({p_x, p_y});
    }

    return smoothed_path;
}

std::vector<Point2D> PathSmoother::smoothPath(
    const std::vector<Point2D>& raw_waypoints,
    size_t num_points,
    SmoothingType type)
{
    if (type == SmoothingType::BSPLINE) {
        return bSplineSmoothing(raw_waypoints, num_points, 3);
    }
    
    return {};
}

} // namespace path_smoothing_control
