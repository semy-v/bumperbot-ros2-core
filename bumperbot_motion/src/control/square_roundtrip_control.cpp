#include "bumperbot_motion/control/square_roundtrip_control.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

#include "tf2/time.h"

namespace bumperbot_motion {

SquareRoundtripControl::SquareRoundtripControl(rclcpp_lifecycle::LifecycleNode& node,
                                               nav2_core::Controller& controller,
                                               std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                               VelocityPublisher::SharedPtr velocity_publisher,
                                               const double controller_frequency,
                                               const double vertex_tolerance,
                                               const double start_tolerance,
                                               const double path_resolution,
                                               TryAcquire try_acquire,
                                               Release release)
    : node_(node),
      controller_(controller),
      tf_buffer_(std::move(tf_buffer)),
      velocity_publisher_(std::move(velocity_publisher)),
      controller_frequency_(controller_frequency),
      vertex_tolerance_(vertex_tolerance),
      start_tolerance_(start_tolerance),
      path_resolution_(path_resolution),
      try_acquire_(std::move(try_acquire)),
      release_(std::move(release)) {
    RCLCPP_INFO(node_.get_logger(),
                "SquareRoundtripControl params:"
                " controller_frequency=%.3f"
                " vertex_tolerance=%.3f"
                " start_tolerance=%.3f"
                " path_resolution=%.3f",
                controller_frequency_, vertex_tolerance_, start_tolerance_, path_resolution_);
}

void SquareRoundtripControl::configure() {
    control_timer_ =
        node_.create_wall_timer(std::chrono::duration<double>(1.0 / controller_frequency_),
                                std::bind(&SquareRoundtripControl::controlLoop, this));
    control_timer_->cancel();

    action_server_ = rclcpp_action::create_server<SquareRoundtrip>(
        &node_, "square_roundtrip",
        std::bind(&SquareRoundtripControl::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&SquareRoundtripControl::handleCancel, this, std::placeholders::_1),
        std::bind(&SquareRoundtripControl::handleAccepted, this, std::placeholders::_1));
}

void SquareRoundtripControl::activate() {
    enabled_.store(true);
}

void SquareRoundtripControl::deactivate() {
    enabled_.store(false);
    if (active_goal_) {
        stopRobot();
        abort("Square roundtrip aborted because motion_control_server was deactivated");
    }
}

void SquareRoundtripControl::cleanup() {
    enabled_.store(false);
    if (control_timer_) {
        control_timer_->cancel();
    }
    active_goal_.reset();
    action_server_.reset();
    control_timer_.reset();
    side_length_ = 0.0;
    target_vertex_index_ = 1;
}

rclcpp_action::GoalResponse SquareRoundtripControl::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const SquareRoundtrip::Goal> goal) {
    if (!enabled_.load()) {
        RCLCPP_WARN(node_.get_logger(), "Rejecting SquareRoundtrip goal: server is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!goal || !std::isfinite(goal->side_length) || goal->side_length <= 0.0) {
        RCLCPP_WARN(node_.get_logger(), "Rejecting SquareRoundtrip goal: side_length must be > 0");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!try_acquire_()) {
        RCLCPP_WARN(node_.get_logger(),
                    "Rejecting SquareRoundtrip goal: another motion action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SquareRoundtripControl::handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
    if (goal_handle != active_goal_) {
        return rclcpp_action::CancelResponse::REJECT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SquareRoundtripControl::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    active_goal_ = goal_handle;
    side_length_ = goal_handle->get_goal()->side_length;

    // Absolute square in the odom frame:
    // (0,0) -> (L,0) -> (L,-L) -> (0,-L) -> (0,0)
    vertices_[0].x = 0.0;
    vertices_[0].y = 0.0;
    vertices_[1].x = side_length_;
    vertices_[1].y = 0.0;
    vertices_[2].x = side_length_;
    vertices_[2].y = -side_length_;
    vertices_[3].x = 0.0;
    vertices_[3].y = -side_length_;
    vertices_[4].x = 0.0;
    vertices_[4].y = 0.0;

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
        abort("Unable to obtain robot pose in odom frame");
        return;
    }

    const double start_distance =
        std::hypot(robot_pose.pose.position.x, robot_pose.pose.position.y);
    if (start_distance > start_tolerance_) {
        RCLCPP_ERROR(node_.get_logger(),
                     "SquareRoundtrip requires the robot to start at odom (0,0); current=(%.3f, "
                     "%.3f), tolerance=%.3f",
                     robot_pose.pose.position.x, robot_pose.pose.position.y, start_tolerance_);
        abort("Robot is not sufficiently close to odom origin (0,0)");
        return;
    }

    target_vertex_index_ = 1;
    setPlanToCurrentVertex();
    control_timer_->reset();

    RCLCPP_INFO(node_.get_logger(), "SquareRoundtrip started: side_length=%.3f m", side_length_);
}

void SquareRoundtripControl::controlLoop() {
    if (!active_goal_) {
        return;
    }

    if (active_goal_->is_canceling()) {
        stopRobot();
        auto result = std::make_shared<SquareRoundtrip::Result>();
        result->success = false;
        result->message = "Square roundtrip canceled";
        active_goal_->canceled(result);
        finishControl();
        return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;
    }

    publishFeedback(robot_pose);

    if (isVertexReached(robot_pose)) {
        RCLCPP_INFO(node_.get_logger(), "SquareRoundtrip reached vertex %zu at (%.3f, %.3f)",
                    target_vertex_index_, vertices_[target_vertex_index_].x,
                    vertices_[target_vertex_index_].y);

        if (target_vertex_index_ == vertices_.size() - 1) {
            stopRobot();
            succeed();
            return;
        }

        ++target_vertex_index_;
        setPlanToCurrentVertex();
        publishFeedback(robot_pose);
        // Do not publish a command from the previous edge on this cycle.
        return;
    }

    const auto cmd_vel =
        controller_.computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist{}, nullptr);
    velocity_publisher_->publish(cmd_vel);
}

bool SquareRoundtripControl::getRobotPose(geometry_msgs::msg::PoseStamped& pose) const {
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

bool SquareRoundtripControl::isVertexReached(const geometry_msgs::msg::PoseStamped& pose) const {
    const auto& vertex = vertices_[target_vertex_index_];
    const double dx = vertex.x - pose.pose.position.x;
    const double dy = vertex.y - pose.pose.position.y;
    return std::hypot(dx, dy) <= vertex_tolerance_;
}

void SquareRoundtripControl::setPlanToCurrentVertex() {
    const auto& from = vertices_[target_vertex_index_ - 1];
    const auto& to = vertices_[target_vertex_index_];
    controller_.setPlan(buildSegmentPlan(from, to));

    RCLCPP_INFO(node_.get_logger(),
                "SquareRoundtrip tracking edge %zu: (%.3f, %.3f) -> (%.3f, %.3f)",
                target_vertex_index_, from.x, from.y, to.x, to.y);
}

nav_msgs::msg::Path SquareRoundtripControl::buildSegmentPlan(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = node_.now();

    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double distance = std::hypot(dx, dy);
    const double yaw = std::atan2(dy, dx);

    const std::size_t intervals =
        std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(distance / path_resolution_)));

    path.poses.reserve(intervals + 1U);
    for (std::size_t i = 0; i <= intervals; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(intervals);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = from.x + t * dx;
        pose.pose.position.y = from.y + t * dy;
        pose.pose.position.z = 0.0;

        // Plan orientation tangent to the current square edge.
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        path.poses.push_back(std::move(pose));
    }

    return path;
}

void SquareRoundtripControl::publishFeedback(
    const geometry_msgs::msg::PoseStamped& robot_pose) const {
    if (!active_goal_) {
        return;
    }

    auto feedback = std::make_shared<SquareRoundtrip::Feedback>();
    feedback->current_vertex = static_cast<std::uint32_t>(target_vertex_index_);
    feedback->current_pose = robot_pose;
    active_goal_->publish_feedback(feedback);
}

void SquareRoundtripControl::succeed() {
    auto result = std::make_shared<SquareRoundtrip::Result>();
    result->success = true;
    result->message = "Completed square roundtrip";
    active_goal_->succeed(result);
    finishControl();
}

void SquareRoundtripControl::abort(const char* message) {
    if (!active_goal_) {
        return;
    }

    stopRobot();
    auto result = std::make_shared<SquareRoundtrip::Result>();
    result->success = false;
    result->message = message;
    active_goal_->abort(result);
    finishControl();
}

void SquareRoundtripControl::stopRobot() const {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = node_.now();
    stop.header.frame_id = "base_footprint";
    velocity_publisher_->publish(stop);
}

void SquareRoundtripControl::finishControl() {
    active_goal_.reset();
    side_length_ = 0.0;
    target_vertex_index_ = 1;
    control_timer_->cancel();
    release_();
}

}  // namespace bumperbot_motion
