#ifndef PD_PURE_PURSUIT_PLUGIN
#define PD_PURE_PURSUIT_PLUGIN

#include "bumperbot_motion/plugin/motion_control_plugin.hpp"
#include "bumperbot_motion/core/pd_pure_pursuit_core.hpp"

namespace bumperbot_motion
{

using PDPurePursuitPlugin = MotionControlPlugin<PDPurePursuitCore>;

template<>
void PDPurePursuitPlugin::declareParameters(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& plugin_name
);

template<>
void PDPurePursuitPlugin::loadParameters(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& plugin_name
);

} // namespace bumperbot_motion

#endif // PD_PURE_PURSUIT_PLUGIN
