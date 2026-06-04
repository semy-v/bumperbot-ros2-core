#ifndef PD_PURE_PURSUIT_CORE
#define PD_PURE_PURSUIT_CORE

#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

#include "motion_control_core_concept.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/time.hpp"

namespace bumperbot_motion
{

class PDPurePursuitCore {
public:
  struct Configuration {
    // Default configuration parameters
    double kp{2.0};
    double kd{0.15};
    double lookahead_distance{0.2};
    double max_linear_velocity{0.35};
    double max_angular_velocity{1.0};
    double near_goal_region{0.2};
    double near_goal_linear_velocity{0.05};
  };

  PDPurePursuitCore() = default;
  ~PDPurePursuitCore() = default;

  void setParameters(const Configuration& parameters) {
    config_params_ = parameters;
  }
  
  void setPlan(const nav_msgs::msg::Path & path) {
    transformed_plan_ = path; // store initial path plan
    next_pose_index_ = 0;
    first_cycle_ = true;
  }

  bool computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const rclcpp::Time & current_time,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    geometry_msgs::msg::Twist & cmd_vel_out,
    geometry_msgs::msg::PoseStamped & next_pose_out);

private:
  bool transformPlan(const std::string & frame, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer);
  geometry_msgs::msg::PoseStamped getLookaheadPose(const geometry_msgs::msg::PoseStamped & robot_pose);
  double calculateAngularVelocity(const double heading_error, const double dt);
  double calculateLinearVelocity(const double heading_error, const double distance_error);

  nav_msgs::msg::Path transformed_plan_;
  rclcpp::Time last_cycle_time_;
  double prev_heading_error_{};
  size_t next_pose_index_{};
  bool first_cycle_{false};

  Configuration config_params_{};
};

static_assert(MotionControlCoreConcept<PDPurePursuitCore>,
    "PDPurePursuitCore must fulfill MotionControlCoreConcept concept requirements!");

}  // namespace bumperbot_motion

#endif // PD_PURE_PURSUIT_CORE