#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robonix_nav2_terminal/persistent_goal_checker.hpp"
#include "robonix_nav2_terminal/persistent_rotate_to_goal_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

geometry_msgs::msg::Pose pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose value;
  value.position.x = x;
  value.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  value.orientation = tf2::toMsg(q);
  return value;
}

TEST(PersistentGoalChecker, SameGoalReplanDoesNotClearXyLatch)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test");
  robonix_nav2_terminal::PersistentGoalChecker checker;
  checker.initialize(node, "goal_checker", nullptr);
  const auto goal = pose(0.0, 0.0, 0.0);
  geometry_msgs::msg::Twist stopped;

  EXPECT_FALSE(checker.isGoalReached(pose(0.20, 0.0, 0.5), goal, stopped));
  checker.reset();  // Simulates ControllerServer receiving a 5 Hz replan.
  EXPECT_TRUE(checker.isGoalReached(pose(0.40, 0.0, 0.0), goal, stopped));
}

TEST(PersistentGoalChecker, NewGoalClearsLatch)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("new_goal_test");
  robonix_nav2_terminal::PersistentGoalChecker checker;
  checker.initialize(node, "goal_checker", nullptr);
  geometry_msgs::msg::Twist stopped;
  EXPECT_TRUE(checker.isGoalReached(
    pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0), stopped));
  EXPECT_FALSE(checker.isGoalReached(
    pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0), stopped));
}

TEST(PersistentRotateToGoalCritic, ReplanDoesNotReenableTranslation)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("critic_test");
  robonix_nav2_terminal::PersistentRotateToGoalCritic critic;
  critic.initialize(node, "PersistentRotateToGoal", "FollowPath", nullptr);

  geometry_msgs::msg::Pose2D current;
  current.x = 0.20;
  geometry_msgs::msg::Pose2D goal;
  goal.theta = 1.0;
  nav_2d_msgs::msg::Twist2D stopped;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(current, stopped, goal, path));

  critic.reset();  // DWB setPlan() during a same-goal replan.
  current.x = 0.40;  // Wheel slip moved XY outside the enter window.
  current.theta = 0.10;
  ASSERT_TRUE(critic.prepare(current, stopped, goal, path));

  dwb_msgs::msg::Trajectory2D translating;
  translating.velocity.x = 0.10;
  translating.poses.push_back(current);
  EXPECT_THROW(
    critic.scoreTrajectory(translating),
    dwb_core::IllegalTrajectoryException);
}

TEST(PluginRegistration, LoadsBothTerminalPlugins)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_loader(
    "nav2_core", "nav2_core::GoalChecker");
  EXPECT_NO_THROW(goal_loader.createSharedInstance(
      "robonix_nav2_terminal::PersistentGoalChecker"));

  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critic_loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  EXPECT_NO_THROW(critic_loader.createSharedInstance(
      "robonix_nav2_terminal::PersistentRotateToGoalCritic"));
}

TEST(PluginRegistration, LoadsSystemDwbControllerAlongsideTerminalOverlay)
{
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader(
    "nav2_core", "nav2_core::Controller");
  EXPECT_NO_THROW(controller_loader.createSharedInstance(
      "dwb_core::DWBLocalPlanner"));
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
