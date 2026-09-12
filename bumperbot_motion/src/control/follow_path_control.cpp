#include "bumperbot_motion/control/follow_path_control.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_motion {

FollowPathControl::FollowPathControl(rclcpp_lifecycle::LifecycleNode& node,
                                     nav2_core::Controller& controller,
                                     std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                     VelocityPublisher::SharedPtr velocity_publisher,
                                     const double controller_frequency,
                                     const double goal_tolerance,
                                     TryAcquire try_acquire,
                                     Release release)
    : node_(node),
      controller_(controller),
      tf_buffer_(std::move(tf_buffer)),
      velocity_publisher_(std::move(velocity_publisher)),
      controller_frequency_(controller_frequency),
      goal_tolerance_(goal_tolerance),
      try_acquire_(std::move(try_acquire)),
      release_(std::move(release)) {}

void FollowPathControl::configure() {
    control_timer_ =
        node_.create_wall_timer(std::chrono::duration<double>(1.0 / controller_frequency_),
                                std::bind(&FollowPathControl::controlLoop, this));
    control_timer_->cancel();

    action_server_ = rclcpp_action::create_server<FollowPath>(
        &node_, "follow_path",
        std::bind(&FollowPathControl::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&FollowPathControl::handleCancel, this, std::placeholders::_1),
        std::bind(&FollowPathControl::handleAccepted, this, std::placeholders::_1));
}

void FollowPathControl::activate() {
    enabled_.store(true);
}

void FollowPathControl::deactivate() {
    enabled_.store(false);

    if (active_goal_) {
        stopRobot();
        auto result = std::make_shared<FollowPath::Result>();
        active_goal_->abort(result);
        finishControl();
    }
}

void FollowPathControl::cleanup() {
    enabled_.store(false);
    if (control_timer_) {
        control_timer_->cancel();
    }
    active_goal_.reset();
    current_path_ = nav_msgs::msg::Path{};
    action_server_.reset();
    control_timer_.reset();
}

rclcpp_action::GoalResponse FollowPathControl::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FollowPath::Goal> goal) {
    if (!enabled_.load()) {
        RCLCPP_WARN(node_.get_logger(), "Rejecting FollowPath goal: server is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!goal || goal->path.poses.empty()) {
        RCLCPP_WARN(node_.get_logger(), "Rejecting FollowPath goal: path is empty");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!try_acquire_()) {
        RCLCPP_WARN(node_.get_logger(),
                    "Rejecting FollowPath goal: another motion action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowPathControl::handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
    if (goal_handle != active_goal_) {
        return rclcpp_action::CancelResponse::REJECT;
    }

    // Do not tear the control loop down here. The loop observes is_canceling(),
    // stops the robot, transitions the goal to CANCELED, and then releases ownership.
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowPathControl::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    active_goal_ = goal_handle;
    current_path_ = goal_handle->get_goal()->path;

    controller_.setPlan(current_path_);
    control_timer_->reset();
}

void FollowPathControl::controlLoop() {
    if (!active_goal_) {
        return;
    }

    auto result = std::make_shared<FollowPath::Result>();

    if (active_goal_->is_canceling()) {
        stopRobot();
        active_goal_->canceled(result);
        finishControl();
        return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;
    }

    // Check before computing/publishing another command once the terminal pose
    // is already within tolerance.
    if (isGoalReached(robot_pose)) {
        stopRobot();
        active_goal_->succeed(result);
        finishControl();
        return;
    }

    const auto cmd_vel =
        controller_.computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist{}, nullptr);
    velocity_publisher_->publish(cmd_vel);
}

bool FollowPathControl::getRobotPose(geometry_msgs::msg::PoseStamped& pose) const {
    try {
        const auto transform =
            tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

        pose.header.frame_id = "odom";
        pose.header.stamp = transform.header.stamp;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to get robot pose: %s", ex.what());
        return false;
    }
}

bool FollowPathControl::isGoalReached(const geometry_msgs::msg::PoseStamped& pose) const {
    const auto& goal = current_path_.poses.back();
    const double dx = goal.pose.position.x - pose.pose.position.x;
    const double dy = goal.pose.position.y - pose.pose.position.y;
    return std::hypot(dx, dy) <= goal_tolerance_;
}

void FollowPathControl::stopRobot() const {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = node_.now();
    stop.header.frame_id = "base_footprint";
    velocity_publisher_->publish(stop);
}

void FollowPathControl::finishControl() {
    active_goal_.reset();
    current_path_ = nav_msgs::msg::Path{};
    control_timer_->cancel();
    release_();
}

}  // namespace bumperbot_motion