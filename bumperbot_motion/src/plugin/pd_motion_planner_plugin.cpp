#include "bumperbot_motion/plugin/pd_motion_planner_plugin.hpp"
#include "nav2_util/node_utils.hpp"

namespace bumperbot_motion
{

template<>
void PDMotionPlannerPlugin::declareParameters(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& plugin_name)
{
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".kp", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".kd", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".step_size", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
}

template<>
void PDMotionPlannerPlugin::loadParameters(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& plugin_name)
{
  core_.setParameters(
    node->get_parameter(plugin_name + ".kp").as_double(),
    node->get_parameter(plugin_name + ".kd").as_double(),
    node->get_parameter(plugin_name + ".step_size").as_double(),
    node->get_parameter(plugin_name + ".max_linear_velocity").as_double(),
    node->get_parameter(plugin_name + ".max_angular_velocity").as_double());
}

}  // namespace bumperbot_motion

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bumperbot_motion::PDMotionPlannerPlugin,
  nav2_core::Controller)