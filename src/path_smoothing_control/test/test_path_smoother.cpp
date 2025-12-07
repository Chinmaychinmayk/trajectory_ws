#include <gtest/gtest.h>
#include "path_smoothing_control/path_smoother.hpp"
#include <vector>

using namespace path_smoothing_control;

class PathSmootherTest : public ::testing::Test {
protected:
    PathSmoother smoother_;
    std::vector<Point2D> waypoints_;

    void SetUp() override {
        waypoints_ = {
            {0.0, 0.0},
            {1.0, 1.0},
            {2.0, 0.0},
            {3.0, 1.0},
            {4.0, 0.0}
        };
    }
};

TEST_F(PathSmootherTest, BSplineSampleCount) {
    size_t num_samples = 20;
    auto path = smoother_.smoothPath(waypoints_, num_samples, SmoothingType::BSPLINE);
    EXPECT_EQ(path.size(), num_samples);
}

TEST_F(PathSmootherTest, StartAndEndPoints) {
    auto path = smoother_.smoothPath(waypoints_, 20, SmoothingType::BSPLINE);
    ASSERT_FALSE(path.empty());
    
    // B-Spline endpoint interpolation depends on knot vector
    // With clamped uniform knots (start/end knots repeated k+1 times), 
    // the curve should pass through the first and last control points.
    
    // Check first point
    EXPECT_NEAR(path.front().x, waypoints_.front().x, 1e-6);
    EXPECT_NEAR(path.front().y, waypoints_.front().y, 1e-6);

    // Check last point
    EXPECT_NEAR(path.back().x, waypoints_.back().x, 1e-6);
    EXPECT_NEAR(path.back().y, waypoints_.back().y, 1e-6);
}

TEST_F(PathSmootherTest, InsufficientPoints) {
    std::vector<Point2D> short_path = {{0.0, 0.0}, {1.0, 1.0}};
    // Should fallback to original points because degree defaults to 3 and we have < 4 points,
    // BUT the implementation adjusts degree for small paths:
    // if size <= 3, degree = size - 1.
    // For size 2, degree = 1 (linear). 
    // n = 1. knots size = 2 + 1 + 1 = 4.
    
    auto path = smoother_.smoothPath(short_path, 10, SmoothingType::BSPLINE);
    EXPECT_EQ(path.size(), 10u);
    
    // Should be a straight line
    EXPECT_NEAR(path[5].x, 0.555, 0.1); // approx check
}

TEST_F(PathSmootherTest, EmptyInput) {
    std::vector<Point2D> empty;
    auto path = smoother_.smoothPath(empty, 10, SmoothingType::BSPLINE);
    EXPECT_TRUE(path.empty());
}
