#include "rclcpp/rclcpp.hpp"
#include "bumperbot_motion/motion_control_server.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_motion::MotionControlServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}