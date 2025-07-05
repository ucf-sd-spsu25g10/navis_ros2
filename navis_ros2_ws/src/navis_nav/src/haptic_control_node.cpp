#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "navis_msgs/msg/control_out.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class HapticControlNode : public rclcpp::Node {
public:
    HapticControlNode() : Node("haptic_control_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&HapticControlNode::odom_callback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan",
            10,
            std::bind(&HapticControlNode::path_callback, this, std::placeholders::_1));

        control_pub_ = this->create_publisher<navis_msgs::msg::ControlOut>("/control_output", 10);

        // Timer to run the control logic at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HapticControlNode::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = msg;
    }

    void control_loop() {
        if (!current_pose_ || !current_path_ || current_path_->poses.empty()) {
            // If we don't have pose or path, do nothing.
            // Optionally, send a "neutral" haptic signal.
            publish_haptic_feedback(0);
            return;
        }

        // Get current position and orientation
        auto current_position = current_pose_->position;
        tf2::Quaternion q(
            current_pose_->orientation.x,
            current_pose_->orientation.y,
            current_pose_->orientation.z,
            current_pose_->orientation.w);
        tf2::Matrix3x3 m(q);
        double current_roll, current_pitch, current_yaw;
        m.getRPY(current_roll, current_pitch, current_yaw);

        // Find the next waypoint to target.
        // A simple approach is to use the next pose in the path.
        // A more robust approach would find the closest point on the path.
        // For now, we'll use the second pose in the list if available, as the first is often the current location.
        geometry_msgs::msg::Point target_position;
        if (current_path_->poses.size() > 1) {
            target_position = current_path_->poses[1].pose.position;
        } else {
            target_position = current_path_->poses[0].pose.position;
        }


        // Calculate the angle to the target waypoint
        double target_angle = atan2(
            target_position.y - current_position.y,
            target_position.x - current_position.x);

        // Calculate the difference between our heading and the target heading
        double angle_error = target_angle - current_yaw;

        // Normalize the angle to be between -PI and PI
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        // Scale the angle error to the haptic output range [-255, 255]
        // angle_error is in radians, from -PI to PI.
        int haptic_output = static_cast<int>((angle_error / M_PI) * 255.0);

        publish_haptic_feedback(haptic_output);
    }

    void publish_haptic_feedback(int value) {
        auto control_msg = navis_msgs::msg::ControlOut();
        control_msg.buzzer_strength = value;
        control_pub_->publish(control_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<geometry_msgs::msg::Pose> current_pose_;
    nav_msgs::msg::Path::SharedPtr current_path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticControlNode>());
    rclcpp::shutdown();
    return 0;
}