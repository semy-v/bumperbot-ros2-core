#include "bumperbot_motion/core/pd_pure_pursuit_core.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>

namespace bumperbot_motion
{

bool PDPurePursuitCore::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const rclcpp::Time & current_time,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  geometry_msgs::msg::Twist & cmd_vel_out,
  geometry_msgs::msg::PoseStamped & next_pose_out)
{
  // ------------------------------------------------------------
  // Validate plan
  // ------------------------------------------------------------
  if(transformed_plan_.poses.empty()) {
    return false;
  }

  // ------------------------------------------------------------
  // Transform plan to robot's frame at first cycle
  // ------------------------------------------------------------
  if (first_cycle_) {
    if(!transformPlan(robot_pose.header.frame_id, tf_buffer)) {
      return false;
    }
  }

  // ------------------------------------------------------------
  // Select target pose
  // ------------------------------------------------------------
  next_pose_out = getLookaheadPose(robot_pose);
  
  // ------------------------------------------------------------
  // Transform target pose into robot frame coordinates
  // ------------------------------------------------------------
  tf2::Transform robot_tf, target_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(next_pose_out.pose, target_tf);
  tf2::Transform relative_tf = robot_tf.inverse() * target_tf;

  // Relative target coordinates in robot frame
  const double x = relative_tf.getOrigin().x();
  const double y = relative_tf.getOrigin().y();

  // ------------------------------------------------------------
  // Compute errors
  // ------------------------------------------------------------
  // Distance to target
  const double distance_error = std::hypot(x, y);

  // Heading error
  //
  // atan2(y,x):
  //   > 0 -> target on left
  //   < 0 -> target on right
  //
  const double heading_error = std::atan2(y, x);

  // ------------------------------------------------------------
  // Time delta
  // ------------------------------------------------------------
  const double dt = first_cycle_
    ? 0.1 
    : std::max((current_time - last_cycle_time_).seconds(), 1e-4);

  // ------------------------------------------------------------
  // Angular velocity control
  // ------------------------------------------------------------
  cmd_vel_out.angular.z = calculateAngularVelocity(heading_error, dt);

  // ------------------------------------------------------------
  // Linear velocity control
  // ------------------------------------------------------------
  cmd_vel_out.linear.x = calculateLinearVelocity(heading_error, distance_error);

  // ------------------------------------------------------------
  // Store controller state
  // ------------------------------------------------------------
  prev_heading_error_ = heading_error;
  last_cycle_time_ = current_time;
  first_cycle_ = false;

  return true;
}

double PDPurePursuitCore::calculateAngularVelocity(const double heading_error, const double dt) {
  // Derivative term
  const double heading_error_derivative =
    (heading_error - prev_heading_error_) / dt;
  
  // Clamped result angular velocity
  return std::clamp(
    config_params_.kp * heading_error + config_params_.kd * heading_error_derivative,
    -config_params_.max_angular_velocity,
    config_params_.max_angular_velocity);
}

double PDPurePursuitCore::calculateLinearVelocity(const double heading_error, const double distance_error) {
  //
  // Stop forward motion if heading error too large.
  //
  constexpr double kMaxHeadingForForwardMotion = M_PI / 2.0;

  if (std::fabs(heading_error) > kMaxHeadingForForwardMotion) {
    return 0.0;
  }

  //
  // Heading-based scaling:
  //
  // - Move faster when target is in front
  // - Slow down when target is sideways
  // - Prevent aggressive forward motion during turns
  //
  // cos(heading_error):
  //
  //   0 deg   -> 1.0
  //   90 deg  -> 0.0
  //   180 deg -> -1.0
  //
  // Clamp negative values to zero
  // to avoid backward motion.
  //
  const double heading_scale = std::max(0.0, std::cos(heading_error));

  //
  // Distance-based scaling
  //
  // This avoids:
  // - overshooting near goal
  // - oscillations
  // - aggressive acceleration
  //
  const double distance_scale = std::clamp(
    distance_error / config_params_.lookahead_distance,
    0.0,
    1.0);

  // ------------------------------------------------------------
  // Base velocity
  // ------------------------------------------------------------
  double linear_velocity =
    config_params_.max_linear_velocity * heading_scale * distance_scale;

  // ------------------------------------------------------------
  // Smooth near-goal regulation
  // ------------------------------------------------------------
  if (distance_error < config_params_.near_goal_region) {
    linear_velocity = std::max(
        linear_velocity,
        config_params_.near_goal_linear_velocity);
  }

  //
  // Clamp result linear velocity
  //
  return std::clamp(
    linear_velocity,
    0.0,
    config_params_.max_linear_velocity
  );
}

geometry_msgs::msg::PoseStamped PDPurePursuitCore::getLookaheadPose(const geometry_msgs::msg::PoseStamped & robot_pose) {
  double accum_dist = 0.0;
  const size_t pose_plan_size = transformed_plan_.poses.size();
  const geometry_msgs::msg::PoseStamped* prev_pose{&robot_pose};

  for (; next_pose_index_ < pose_plan_size; ++next_pose_index_) {
    const auto & next_pose = transformed_plan_.poses[next_pose_index_];
    const double dx = next_pose.pose.position.x - prev_pose->pose.position.x;
    const double dy = next_pose.pose.position.y - prev_pose->pose.position.y;

    accum_dist += std::hypot(dx, dy);

    if (accum_dist >= config_params_.lookahead_distance) {
      return next_pose;
    }

    prev_pose = &next_pose;
  }

  return transformed_plan_.poses.back();
}

bool PDPurePursuitCore::transformPlan(const std::string& target_frame, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer) {
  if(transformed_plan_.header.frame_id == target_frame) {
    return true;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(
      target_frame,
      transformed_plan_.header.frame_id,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return false;
  }

  for(auto & pose : transformed_plan_.poses) {
    tf2::doTransform(pose, pose, tf); // Transforms individual plan poses
  }
  transformed_plan_.header.frame_id = target_frame;

  return true;
}

}  // namespace bumperbot_motion