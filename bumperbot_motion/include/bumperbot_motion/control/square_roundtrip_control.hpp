#ifndef SQUARE_ROUNDTRIP_CONTROL_HPP_
#define SQUARE_ROUNDTRIP_CONTROL_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "bumperbot_msgs/action/square_roundtrip.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace bumperbot_motion {

class SquareRoundtripControl {
 public:
    using SquareRoundtrip = bumperbot_msgs::action::SquareRoundtrip;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SquareRoundtrip>;
    using VelocityPublisher =
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>;
    using TryAcquire = std::function<bool()>;
    using Release = std::function<void()>;

    SquareRoundtripControl(rclcpp_lifecycle::LifecycleNode& node,
                           nav2_core::Controller& controller,
                           std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                           VelocityPublisher::SharedPtr velocity_publisher,
                           double controller_frequency,
                           double vertex_tolerance,
                           double start_tolerance,
                           double path_resolution,
                           TryAcquire try_acquire,
                           Release release);

    void configure();
    void activate();
    void deactivate();
    void cleanup();

 private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const SquareRoundtrip::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

    void controlLoop();
    bool getRobotPose(geometry_msgs::msg::PoseStamped& pose) const;
    bool isVertexReached(const geometry_msgs::msg::PoseStamped& pose) const;
    void setPlanToCurrentVertex();
    nav_msgs::msg::Path buildSegmentPlan(const geometry_msgs::msg::Point& from,
                                         const geometry_msgs::msg::Point& to) const;
    void publishFeedback(const geometry_msgs::msg::PoseStamped& robot_pose) const;
    void succeed();
    void abort(const char* message);
    void stopRobot() const;
    void finishControl();

    rclcpp_lifecycle::LifecycleNode& node_;
    nav2_core::Controller& controller_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    VelocityPublisher::SharedPtr velocity_publisher_;
    double controller_frequency_;
    double vertex_tolerance_;
    double start_tolerance_;
    double path_resolution_;
    TryAcquire try_acquire_;
    Release release_;

    std::atomic_bool enabled_{false};
    rclcpp_action::Server<SquareRoundtrip>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<GoalHandle> active_goal_;

    double side_length_{0.0};
    std::array<geometry_msgs::msg::Point, 5> vertices_{
        geometry_msgs::msg::Point{}, geometry_msgs::msg::Point{}, geometry_msgs::msg::Point{},
        geometry_msgs::msg::Point{}, geometry_msgs::msg::Point{}};
    std::size_t target_vertex_index_{1};
};

}  // namespace bumperbot_motion

#endif  // SQUARE_ROUNDTRIP_CONTROL_HPP_