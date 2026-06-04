#include "bumperbot_motion/plugin/pd_pure_pursuit_plugin.hpp"
#include "nav2_util/node_utils.hpp"

namespace bumperbot_motion
{

template<>
void PDPurePursuitPlugin::declareParameters(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& plugin_name)
{
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".kp", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".kd", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".lookahead_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_linear_velocity", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".near_goal_region", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".near_goal_linear_velocity", rclcpp::ParameterValue(0.05));
}

template<>
void PDPurePursuitPlugin::loadParameters(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& plugin_name)
{
  const decltype(core_)::Configuration parameters{
    .kp = node->get_parameter(plugin_name + ".kp").as_double(),
    .kd = node->get_parameter(plugin_name + ".kd").as_double(),
    .lookahead_distance = node->get_parameter(plugin_name + ".lookahead_distance").as_double(),
    .max_linear_velocity = node->get_parameter(plugin_name + ".max_linear_velocity").as_double(),
    .max_angular_velocity = node->get_parameter(plugin_name + ".max_angular_velocity").as_double(),
    .near_goal_region = node->get_parameter(plugin_name + ".near_goal_region").as_double(),
    .near_goal_linear_velocity = node->get_parameter(plugin_name + ".near_goal_linear_velocity").as_double(),
  };

  RCLCPP_INFO(
    node->get_logger(),
    "%s params:"
    " kp=%.3f"
    " kd=%.3f"
    " lookahead_distance=%.3f"
    " max_linear_velocity=%.3f"
    " max_angular_velocity=%.3f"
    " near_goal_region=%.3f"
    " near_goal_linear_velocity=%.3f",
    plugin_name.c_str(),
    parameters.kp,
    parameters.kd,
    parameters.lookahead_distance,
    parameters.max_linear_velocity,
    parameters.max_angular_velocity,
    parameters.near_goal_region,
    parameters.near_goal_linear_velocity
  );

  core_.setParameters(parameters);
}

}  // namespace bumperbot_motion

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bumperbot_motion::PDPurePursuitPlugin,
  nav2_core::Controller)