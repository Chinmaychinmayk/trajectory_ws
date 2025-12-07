#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include <vector>
#include "nav_msgs/msg/path.hpp"

namespace path_smoothing_control
{
/**
 * @brief Simple 2D point structure.
 */
struct Point2D {
    double x, y;
};

/**
 * @brief Available smoothing types.
 */
enum class SmoothingType {
    BSPLINE = 0,
    BEZIER = 1,
    CUBIC_SPLINE = 2
};

/**
 * @brief Class to implement path smoothing algorithms.
 */
class PathSmoother
{
public:
    PathSmoother() = default;
    ~PathSmoother() = default;

    /**
     * @brief Smooths the given waypoints.
     */
    std::vector<Point2D> smoothPath(
        const std::vector<Point2D>& raw_waypoints,
        size_t num_points,
        SmoothingType type = SmoothingType::BSPLINE
    );

private:
    std::vector<double> generateKnotVector(size_t n, int k);
    double basisFunction(int i, int k, double t, const std::vector<double>& knots);

    // Placeholder for other smoothing methods (not fully implemented in .cpp for brevity)
    std::vector<Point2D> bSplineSmoothing(const std::vector<Point2D>& raw_waypoints, size_t num_points, int degree);
};
} // namespace path_smoothing_control

#endif // PATH_SMOOTHER_HPP
