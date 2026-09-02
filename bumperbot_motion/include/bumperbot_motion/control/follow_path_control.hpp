#ifndef FOLLOW_PATH_CONTROL_HPP_
#define FOLLOW_PATH_CONTROL_HPP_

#include <atomic>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace bumperbot_motion {

class FollowPathControl {
 public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;
    using VelocityPublisher =
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>;
    using TryAcquire = std::function<bool()>;
    using Release = std::function<void()>;

    FollowPathControl(rclcpp_lifecycle::LifecycleNode& node,
                      nav2_core::Controller& controller,
                      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                      VelocityPublisher::SharedPtr velocity_publisher,
                      double controller_frequency,
                      double goal_tolerance,
                      TryAcquire try_acquire,
                      Release release);

    void configure();
    void activate();
    void deactivate();
    void cleanup();

 private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const FollowPath::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

    void controlLoop();
    bool getRobotPose(geometry_msgs::msg::PoseStamped& pose) const;
    bool isGoalReached(const geometry_msgs::msg::PoseStamped& pose) const;
    void stopRobot() const;
    void finishControl();

    rclcpp_lifecycle::LifecycleNode& node_;
    nav2_core::Controller& controller_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    VelocityPublisher::SharedPtr velocity_publisher_;
    double controller_frequency_;
    double goal_tolerance_;
    TryAcquire try_acquire_;
    Release release_;

    std::atomic_bool enabled_{false};
    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<GoalHandle> active_goal_;
    nav_msgs::msg::Path current_path_;
};

}  // namespace bumperbot_motion

#endif  // FOLLOW_PATH_CONTROL_HPP_