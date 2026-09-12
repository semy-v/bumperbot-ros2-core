#ifndef MOTION_CONTROL_SERVER_HPP
#define MOTION_CONTROL_SERVER_HPP

#include <memory>
#include <mutex>
#include <string>

#include "bumperbot_motion/control/follow_path_control.hpp"
#include "bumperbot_motion/control/square_roundtrip_control.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_motion {

class MotionControlServer : public rclcpp_lifecycle::LifecycleNode {
 public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit MotionControlServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
    enum class ActiveControl { kNone, kFollowPath, kSquareRoundtrip };

    bool loadControllerPlugin();
    bool tryAcquireControl(ActiveControl control);
    void releaseControl(ActiveControl control);

    pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
    pluginlib::UniquePtr<nav2_core::Controller> controller_{nullptr};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        vel_publisher_{nullptr};

    std::unique_ptr<FollowPathControl> follow_path_control_;
    std::unique_ptr<SquareRoundtripControl> square_roundtrip_control_;

    std::mutex active_control_mutex_;
    ActiveControl active_control_{ActiveControl::kNone};
};

}  // namespace bumperbot_motion

#endif  // MOTION_CONTROL_SERVER_HPP