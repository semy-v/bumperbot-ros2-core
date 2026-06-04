#ifndef MOTION_CONTROL_SERVER_HPP
#define MOTION_CONTROL_SERVER_HPP

#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace bumperbot_motion
{

class MotionControlServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  MotionControlServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  
  std::optional<std::string> getControllerPluginTypeParam();
  bool loadControllerPlugin();
  void controlLoop();
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & pose);
  void stopControlLoop();
  void stopRobot();

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  rclcpp_action::Server<FollowPath>::SharedPtr action_server_{nullptr};
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_publisher_{nullptr};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  pluginlib::UniquePtr<nav2_core::Controller> controller_{nullptr};
  rclcpp::TimerBase::SharedPtr control_timer_{nullptr};
  std::shared_ptr<GoalHandleFollowPath> active_goal_{nullptr};

  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
  nav_msgs::msg::Path current_path_;
};

}  // namespace bumperbot_motion

#endif // MOTION_CONTROL_SERVER_HPP