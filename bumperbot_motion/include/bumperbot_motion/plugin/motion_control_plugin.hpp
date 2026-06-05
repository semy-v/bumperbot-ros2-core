#ifndef MOTION_CONTROL_PLUGIN_HPP
#define MOTION_CONTROL_PLUGIN_HPP

#include "bumperbot_motion/core/motion_control_core_concept.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bumperbot_motion
{

template<MotionControlCoreConcept CoreT>
class MotionControlPlugin : public nav2_core::Controller
{
public:
  MotionControlPlugin() = default;
  ~MotionControlPlugin() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Failed to lock node");
    }

    tf_buffer_ = tf;
    clock_ = node->get_clock();

    declareParameters(node, name);
    loadParameters(node, name);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    core_.setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override
  {
    geometry_msgs::msg::TwistStamped out;
    out.header.frame_id = pose.header.frame_id;
    out.header.stamp = clock_->now();

    geometry_msgs::msg::PoseStamped next_pose;
    bool ok = core_.computeVelocityCommands(
      pose, out.header.stamp, tf_buffer_, out.twist, next_pose);

    if (not ok) {
       out.twist = geometry_msgs::msg::Twist();
    }

    return out;
  }

  void setSpeedLimit(const double &/*speed_limit*/, const bool &/*percentage*/) override {
    // Implementation of dynamic speed limits omitted for brevity.
  }

protected:
  // ---- Parameter hooks (customizable per motion control core) ----
  void declareParameters(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& /*node*/,
    const std::string& /*plugin_name*/)
  {
    // specialize per motion control core plugin type
  }

  void loadParameters(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& /*node*/,
    const std::string& /*plugin_name*/)
  {
    // specialize per motion control core plugin type
  }

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Clock::SharedPtr clock_;
  
  // motion control core algorithm implementation
  CoreT core_{};
};

} // namespace bumperbot_motion


#endif // MOTION_CONTROL_PLUGIN_HPP
