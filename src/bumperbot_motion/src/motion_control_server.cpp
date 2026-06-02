#include "bumperbot_motion/motion_control_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace bumperbot_motion
{

MotionControlServer::MotionControlServer(const rclcpp::NodeOptions & options)
  : LifecycleNode("motion_control_server", options)
  , controller_loader_("nav2_core", "nav2_core::Controller")
{
}

auto MotionControlServer::on_configure(const rclcpp_lifecycle::State &) -> CallbackReturn
{
  if (false == loadControllerPlugin()) {
    return CallbackReturn::FAILURE;
  }

  declare_parameter("controller_frequency", 20.0);
  const double controller_frequency = get_parameter("controller_frequency").as_double();
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / controller_frequency),
    std::bind(&MotionControlServer::controlLoop, this));
  control_timer_->cancel();  // control loop started at action accept 

  declare_parameter("cmd_velocity_topic", "/bumperbot_controller/cmd_vel");
  const std::string cmd_velocity_topic = get_parameter("cmd_velocity_topic").as_string();
  vel_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_velocity_topic, 10);

  action_server_ = rclcpp_action::create_server<FollowPath>(
    this, "follow_path",
    std::bind(&MotionControlServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionControlServer::handleCancel, this, std::placeholders::_1),
    std::bind(&MotionControlServer::handleAccepted, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_activate(const rclcpp_lifecycle::State &) -> CallbackReturn {
  vel_publisher_->on_activate();
  controller_->activate();
  return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_deactivate(const rclcpp_lifecycle::State &) -> CallbackReturn {
  stopRobot();
  stopControlLoop();
  vel_publisher_->on_deactivate();
  controller_->deactivate();
  return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_cleanup(const rclcpp_lifecycle::State &) -> CallbackReturn {
  control_timer_.reset();
  active_goal_.reset();
  action_server_.reset();
  vel_publisher_.reset();
  controller_->cleanup();
  controller_.reset();
  return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_shutdown(const rclcpp_lifecycle::State &) -> CallbackReturn {
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse MotionControlServer::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal>)
{
  RCLCPP_INFO(get_logger(), "handleGoal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlServer::handleCancel(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(get_logger(), "handleCancel");
  if (goal_handle == active_goal_) {
    stopRobot();
    stopControlLoop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlServer::handleAccepted(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(get_logger(), "handleAccepted");
  active_goal_ = goal_handle;
  current_path_ = goal_handle->get_goal()->path;

  controller_->setPlan(current_path_);

  control_timer_->reset();  // start control loop
}

bool MotionControlServer::loadControllerPlugin() {
  declare_parameter("controller_plugin", "pd_pure_pursuit");
  const std::string controller_plugin = get_parameter("controller_plugin").as_string();

  const std::string plugin_type_param = controller_plugin + ".plugin";
  declare_parameter<std::string>(plugin_type_param, "");
  const std::string controller_plugin_type = get_parameter(plugin_type_param).as_string();

  if (controller_plugin_type.empty()) {
    RCLCPP_ERROR(get_logger(), "Controller plugin type parameter missing: %s",
      plugin_type_param.c_str());
    return false;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  try {
    controller_ = controller_loader_.createUniqueInstance(controller_plugin_type);
    controller_->configure(weak_from_this(), controller_plugin_type, tf_buffer_, nullptr/*no costmap*/);
    RCLCPP_INFO(get_logger(), "Loaded controller plugin: %s", controller_plugin_type.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load controller plugin: %s", ex.what());
    return false;
  }

  return true;
}

void MotionControlServer::controlLoop() {
    if (!active_goal_) {
    return;
  }

  auto result = std::make_shared<FollowPath::Result>();

  // Cancel check
  if (active_goal_->is_canceling()) {
    stopRobot();
    active_goal_->canceled(result);
    stopControlLoop();
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;

  if (!getRobotPose(robot_pose)) {
    return;
  }

  auto cmd_vel = controller_->computeVelocityCommands(
    robot_pose,
    geometry_msgs::msg::Twist{}, /*current velocity not used*/
    nullptr/*goal checker*/);

  // Completion check
  if (isGoalReached(robot_pose)) {
    stopRobot();
    active_goal_->succeed(result);
    stopControlLoop();
    return;
  }

  vel_publisher_->publish(cmd_vel);
}

bool MotionControlServer::isGoalReached(
  const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & goal = current_path_.poses.back();

  double dx = goal.pose.position.x - pose.pose.position.x;
  double dy = goal.pose.position.y - pose.pose.position.y;

  return std::hypot(dx, dy) <= 0.05;
}

void MotionControlServer::stopControlLoop() {
  active_goal_.reset();
  control_timer_->cancel();
}

void MotionControlServer::stopRobot() {
  vel_publisher_->publish(geometry_msgs::msg::TwistStamped());
}

bool MotionControlServer::getRobotPose(geometry_msgs::msg::PoseStamped& pose) {
  try {
    auto transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero); // Looks up current position in odom frame
    pose.header.frame_id = "odom";
    pose.header.stamp = transform.header.stamp;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to get robot pose: %s", ex.what());
    return false;
  }
}

}  // namespace bumperbot_motion