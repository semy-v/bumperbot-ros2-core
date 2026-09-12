#include "bumperbot_motion/motion_control_server.hpp"

#include <functional>
#include <string>

namespace bumperbot_motion {

MotionControlServer::MotionControlServer(const rclcpp::NodeOptions& options)
    : LifecycleNode("motion_control_server", options),
      controller_loader_("nav2_core", "nav2_core::Controller") {}

auto MotionControlServer::on_configure(const rclcpp_lifecycle::State&) -> CallbackReturn {
    if (!loadControllerPlugin()) {
        return CallbackReturn::FAILURE;
    }

    declare_parameter("controller_frequency", 20.0);
    const double controller_frequency = get_parameter("controller_frequency").as_double();
    if (controller_frequency <= 0.0) {
        RCLCPP_ERROR(get_logger(), "controller_frequency must be > 0");
        return CallbackReturn::FAILURE;
    }

    declare_parameter("goal_tolerance", 0.05);
    const double goal_tolerance = get_parameter("goal_tolerance").as_double();

    declare_parameter("square_roundtrip.vertex_tolerance", 0.03);
    declare_parameter("square_roundtrip.start_tolerance", 0.10);
    declare_parameter("square_roundtrip.path_resolution", 5.00);
    const double square_vertex_tolerance =
        get_parameter("square_roundtrip.vertex_tolerance").as_double();
    const double square_start_tolerance =
        get_parameter("square_roundtrip.start_tolerance").as_double();
    const double square_path_resolution =
        get_parameter("square_roundtrip.path_resolution").as_double();

    if (goal_tolerance <= 0.0 || square_vertex_tolerance <= 0.0 || square_start_tolerance < 0.0 ||
        square_path_resolution <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Invalid motion-control tolerance/resolution parameter");
        return CallbackReturn::FAILURE;
    }

    declare_parameter("cmd_velocity_topic", "/diff_drive_controller/cmd_vel");
    const std::string cmd_velocity_topic = get_parameter("cmd_velocity_topic").as_string();
    vel_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_velocity_topic, 10);

    follow_path_control_ = std::make_unique<FollowPathControl>(
        *this, *controller_, tf_buffer_, vel_publisher_, controller_frequency, goal_tolerance,
        [this]() { return tryAcquireControl(ActiveControl::kFollowPath); },
        [this]() { releaseControl(ActiveControl::kFollowPath); });
    follow_path_control_->configure();

    square_roundtrip_control_ = std::make_unique<SquareRoundtripControl>(
        *this, *controller_, tf_buffer_, vel_publisher_, controller_frequency,
        square_vertex_tolerance, square_start_tolerance, square_path_resolution,
        [this]() { return tryAcquireControl(ActiveControl::kSquareRoundtrip); },
        [this]() { releaseControl(ActiveControl::kSquareRoundtrip); });
    square_roundtrip_control_->configure();

    return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_activate(const rclcpp_lifecycle::State&) -> CallbackReturn {
    vel_publisher_->on_activate();
    controller_->activate();
    follow_path_control_->activate();
    square_roundtrip_control_->activate();
    return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_deactivate(const rclcpp_lifecycle::State&) -> CallbackReturn {
    // Abort any active action while the velocity publisher is still active.
    follow_path_control_->deactivate();
    square_roundtrip_control_->deactivate();
    controller_->deactivate();
    vel_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_cleanup(const rclcpp_lifecycle::State&) -> CallbackReturn {
    if (follow_path_control_) {
        follow_path_control_->cleanup();
        follow_path_control_.reset();
    }

    if (square_roundtrip_control_) {
        square_roundtrip_control_->cleanup();
        square_roundtrip_control_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(active_control_mutex_);
        active_control_ = ActiveControl::kNone;
    }

    vel_publisher_.reset();

    if (controller_) {
        controller_->cleanup();
        controller_.reset();
    }

    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

auto MotionControlServer::on_shutdown(const rclcpp_lifecycle::State&) -> CallbackReturn {
    return CallbackReturn::SUCCESS;
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
        controller_->configure(weak_from_this(), controller_plugin_type, tf_buffer_,
                               nullptr);  // no costmap
        RCLCPP_INFO(get_logger(), "Loaded controller plugin: %s", controller_plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(get_logger(), "Failed to load controller plugin: %s", ex.what());
        return false;
    }

    return true;
}

bool MotionControlServer::tryAcquireControl(const ActiveControl control) {
    std::lock_guard<std::mutex> lock(active_control_mutex_);
    if (active_control_ != ActiveControl::kNone) {
        return false;
    }

    active_control_ = control;
    return true;
}

void MotionControlServer::releaseControl(const ActiveControl control) {
    std::lock_guard<std::mutex> lock(active_control_mutex_);
    if (active_control_ == control) {
        active_control_ = ActiveControl::kNone;
    }
}

}  // namespace bumperbot_motion
