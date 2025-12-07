#include <gtest/gtest.h>
#include "path_smoothing_control/trajectory_controller.hpp"
#include <cmath>

using namespace path_smoothing_control;

class TrajectoryControllerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        controller_ = std::make_shared<TrajectoryController>(0.3, 0.5, 2.0);
        controller_->setGains(1.0, 2.0, 0.0, 0.1);

        test_trajectory_ = {
            TrajectoryPoint{0.0, 0.0, 0.0, 0.0, 0.0},
            TrajectoryPoint{0.5, 0.0, 0.0, 0.2, 2.5},
            TrajectoryPoint{1.0, 0.0, 0.0, 0.4, 5.0},
            TrajectoryPoint{1.5, 0.5, M_PI/4.0, 0.4, 7.5},
            TrajectoryPoint{2.0, 1.0, M_PI/2.0, 0.0, 10.0}
        };
    }

    std::shared_ptr<TrajectoryController> controller_;
    std::vector<TrajectoryPoint> test_trajectory_;
};

TEST_F(TrajectoryControllerTest, PurePursuitControl)
{
    RobotPose pose(0.1, 0.0, 0.0);
    auto cmd = controller_->computeControl(
        pose, test_trajectory_, 0.5,
        ControllerType::PURE_PURSUIT);

    EXPECT_GE(cmd.linear.x, 0.0);
    EXPECT_LE(cmd.linear.x, 0.5);
    EXPECT_LE(std::abs(cmd.angular.z), 0.5);
}

TEST_F(TrajectoryControllerTest, StanleyControl)
{
    RobotPose pose(0.1, 0.05, 0.1);
    auto cmd = controller_->computeControl(
        pose, test_trajectory_, 0.5,
        ControllerType::STANLEY);

    EXPECT_GE(cmd.linear.x, 0.0);
    EXPECT_LE(cmd.linear.x, 0.5);
    EXPECT_LE(std::abs(cmd.angular.z), 2.0);
    EXPECT_GT(std::abs(cmd.angular.z), 0.05);
}

TEST_F(TrajectoryControllerTest, PIDControl)
{
    RobotPose pose(0.1, 0.1, 0.0);
    auto cmd = controller_->computeControl(
        pose, test_trajectory_, 0.5,
        ControllerType::PID);

    EXPECT_GE(cmd.linear.x, 0.0);
    EXPECT_LE(std::abs(cmd.angular.z), 2.0);
}

TEST_F(TrajectoryControllerTest, GoalReachedCheck)
{
    RobotPose pose(1.95, 0.95, M_PI/2.0 + 0.05);
    TrajectoryPoint goal = test_trajectory_.back();
    bool reached = controller_->isGoalReached(pose, goal, 0.1, 0.2);

    EXPECT_TRUE(reached);
}

TEST_F(TrajectoryControllerTest, EmptyTrajectory)
{
    std::vector<TrajectoryPoint> empty;
    RobotPose pose(0.0, 0.0, 0.0);
    auto cmd = controller_->computeControl(pose, empty, 0.0, ControllerType::PURE_PURSUIT);

    EXPECT_EQ(cmd.linear.x, 0.0);
    EXPECT_EQ(cmd.angular.z, 0.0);
}

TEST_F(TrajectoryControllerTest, TrackingError)
{
    RobotPose pose(0.2, 0.1, 0.05);
    controller_->computeControl(pose, test_trajectory_, 1.0, ControllerType::PURE_PURSUIT);
    
    double error = controller_->getTrackingError();
    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, 1.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
