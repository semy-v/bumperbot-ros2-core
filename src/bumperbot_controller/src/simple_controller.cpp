#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>


using namespace std::placeholders;


class SimpleController : public rclcpp::Node {
public:
    SimpleController()
        : Node("simple_controller")
        , vel_sub_{create_subscription<geometry_msgs::msg::TwistStamped>(
            "bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1))}
        , wheel_cmd_pub_{create_publisher<std_msgs::msg::Float64MultiArray>("simple_velocity_controller/commands", 10)}
        , joint_sub_{create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1))}
        , odom_pub_{create_publisher<nav_msgs::msg::Odometry>(
            "bumperbot_controller/odom", 10)}
        , tf_broadcaster_{this}
    {
        declare_parameter<double>("wheel_radius", 0.033);
        declare_parameter<double>("wheel_separation", 0.17);

        wheel_radius_ = get_parameter("wheel_radius").as_double();
        wheel_separation_ = get_parameter("wheel_separation").as_double();

        RCLCPP_INFO(get_logger(), "Using wheel_radius: %f", wheel_radius_);
        RCLCPP_INFO(get_logger(), "Using wheel_separation: %f", wheel_separation_);

        prev_time_ = get_clock()->now();

        speed_convertion_matrix_ <<
            wheel_radius_/2,                   wheel_radius_/2,
            wheel_radius_/wheel_separation_,  -wheel_radius_/wheel_separation_;

        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_footprint";

        dynamic_transform_stamped_.header.frame_id = "odom";
        dynamic_transform_stamped_.child_frame_id = "base_footprint";

        RCLCPP_INFO_STREAM(get_logger(), "Speed conversion matrix: " << speed_convertion_matrix_);
    }


private:
    void velCallback(const geometry_msgs::msg::TwistStamped& msg) {
        const Eigen::Vector2d robot_speed{msg.twist.linear.x, msg.twist.angular.z};
        const Eigen::Vector2d wheel_speed{speed_convertion_matrix_.inverse() * robot_speed};

        std_msgs::msg::Float64MultiArray wheel_speed_msg;
        wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
        wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

        wheel_cmd_pub_->publish(wheel_speed_msg);
    }

    void jointCallback(const sensor_msgs::msg::JointState& msg) {
        // Calculate change in wheel positions (radians)
        const double dp_left = msg.position.at(0) - left_wheel_prev_pos_;
        const double dp_right = msg.position.at(1) - right_wheel_prev_pos_;
        
        // Calculate instantenous velocity of the wheels
        const auto [fi_left, fi_right] = [&, this]() {
            if (msg.velocity.empty()) {
                rclcpp::Time msg_time = msg.header.stamp;
                double dt_sec = (msg_time - prev_time_).seconds();
                if (dt_sec <= 0) {
                    RCLCPP_WARN(get_logger(), "jointCallback - joint state invalid passed time: %f", dt_sec);
                    throw std::runtime_error{"joint state invalid passed time"};
                }

                prev_time_ = msg_time;

                return std::pair<double,double>{
                    dp_left / dt_sec,
                    dp_right / dt_sec
                };
            } else {
                return std::pair<double,double>{
                    msg.velocity.at(0),
                    msg.velocity.at(1)
                };
            }
        }();
        
        // Calculate robot linear and angular velocity
        const double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
        const double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

        // Calculate distance traveled by each wheel (meters)
        const double d_left = dp_left * wheel_radius_;
        const double d_right = dp_right * wheel_radius_;

        // Calculate robot linear displacement (ds) and angular change (dtheta)
        const double ds = (d_right + d_left) / 2.0;
        const double d_theta = (d_right - d_left) / wheel_separation_;

        // Update Pose using Midpoint Integration
        x_ += ds * std::cos(theta_ + d_theta / 2.0);
        y_ += ds * std::sin(theta_ + d_theta / 2.0);
        theta_ += d_theta;
        
        // Update previous state for next iteration
        left_wheel_prev_pos_ = msg.position.at(0);
        right_wheel_prev_pos_ = msg.position.at(1);

        // calculate Quaterion rotation from Euler
        auto quate =
            tf2::toMsg(tf2::Quaternion::createFromEuler(.0, .0, theta_));

        publishOdometry(quate, linear, angular);
        publishTransform(std::move(quate));
    }

    void publishOdometry(
            geometry_msgs::msg::Quaternion quate,
            const double linear,
            const double angular) {
        // Fill the Pose (Position)
        odom_msg_.pose.pose.position.x = x_;
        odom_msg_.pose.pose.position.y = y_;
        odom_msg_.pose.pose.position.z = 0.0;

        // Fill the Pose (Orientation) with Quaternion
        odom_msg_.pose.pose.orientation = std::move(quate);

        // Fill the Twist (Velocity)
        odom_msg_.twist.twist.linear.x = linear;
        odom_msg_.twist.twist.angular.z = angular;
        
        odom_msg_.header.stamp = get_clock()->now();
        odom_pub_->publish(odom_msg_);
    }

    void publishTransform(geometry_msgs::msg::Quaternion quate) {
        dynamic_transform_stamped_.transform.translation.x = x_;
        dynamic_transform_stamped_.transform.translation.y = y_;
        dynamic_transform_stamped_.transform.rotation = std::move(quate);
        dynamic_transform_stamped_.header.stamp = get_clock()->now();
        tf_broadcaster_.sendTransform(dynamic_transform_stamped_);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double wheel_radius_;
    double wheel_separation_;
    rclcpp::Time prev_time_;
    Eigen::Matrix2d speed_convertion_matrix_;
    nav_msgs::msg::Odometry odom_msg_;

    double left_wheel_prev_pos_{0.0};
    double right_wheel_prev_pos_{0.0};
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_{};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<SimpleController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}