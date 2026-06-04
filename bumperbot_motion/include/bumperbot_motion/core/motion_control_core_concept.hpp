#ifndef MOTION_CONTROL_CORE_CONCEPT_HPP
#define MOTION_CONTROL_CORE_CONCEPT_HPP

#include <concepts>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace bumperbot_motion
{

template<typename T>
concept MotionControlCoreConcept =
requires(T core,
         const nav_msgs::msg::Path & path,
         const geometry_msgs::msg::PoseStamped & pose,
         const rclcpp::Time & time,
         std::shared_ptr<tf2_ros::Buffer> tf,
         geometry_msgs::msg::Twist & cmd,
         geometry_msgs::msg::PoseStamped & next_pose)
{
  // configuration
  { core.setPlan(path) } -> std::same_as<void>;

  // main computation
  { core.computeVelocityCommands(pose, time, tf, cmd, next_pose) } -> std::same_as<bool>;
};

} // namespace bumperbot_motion

#endif // MOTION_CONTROL_CORE_CONCEPT_HPP