#include <gtest/gtest.h>
#include "path_smoothing_control/trajectory_generator.hpp"
#include <cmath>

using namespace path_smoothing_control;

class TrajectoryGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        generator_ = std::make_shared<TrajectoryGenerator>(1.0, 0.5); // max_v=1.0, max_a=0.5
        
        // Simple straight line path
        path_ = {
            {0.0, 0.0},
            {1.0, 0.0},
            {2.0, 0.0},
            {3.0, 0.0},
            {4.0, 0.0}
        };
    }

    std::shared_ptr<TrajectoryGenerator> generator_;
    std::vector<Point2D> path_;
};

TEST_F(TrajectoryGeneratorTest, OutputSizeMatchesInput) {
    auto traj = generator_->generateTrajectory(path_, VelocityProfile::TRAPEZOIDAL);
    EXPECT_EQ(traj.size(), path_.size());
}

TEST_F(TrajectoryGeneratorTest, ZeroVelocityStartEnd) {
    auto traj = generator_->generateTrajectory(path_, VelocityProfile::TRAPEZOIDAL);
    ASSERT_FALSE(traj.empty());
    EXPECT_NEAR(traj.front().v, 0.0, 1e-6);
    EXPECT_NEAR(traj.back().v, 0.0, 1e-6);
}

TEST_F(TrajectoryGeneratorTest, MaxVelocityConstraint) {
    // Make a long path to ensure max velocity is reached
    std::vector<Point2D> long_path;
    for(int i=0; i<=20; ++i) {
        long_path.push_back({(double)i, 0.0});
    }
    
    auto traj = generator_->generateTrajectory(long_path, VelocityProfile::TRAPEZOIDAL);
    
    double max_v_observed = 0.0;
    for(const auto& p : traj) {
        max_v_observed = std::max(max_v_observed, p.v);
    }
    
    EXPECT_LE(max_v_observed, 1.0 + 1e-6);
}

TEST_F(TrajectoryGeneratorTest, MonotonicTime) {
    auto traj = generator_->generateTrajectory(path_, VelocityProfile::TRAPEZOIDAL);
    for(size_t i=1; i<traj.size(); ++i) {
        EXPECT_GT(traj[i].timestamp, traj[i-1].timestamp);
    }
}

TEST_F(TrajectoryGeneratorTest, Headings) {
    // Path moves in +X direction, headings should be 0
    auto traj = generator_->generateTrajectory(path_, VelocityProfile::TRAPEZOIDAL);
    for(const auto& p : traj) {
        EXPECT_NEAR(p.heading, 0.0, 1e-6);
    }
}
