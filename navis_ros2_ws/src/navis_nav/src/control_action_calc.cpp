#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "navis_msgs/msg/control_out.hpp"
#include "navis_nav/audio_mappings.hpp"
#include <cmath>
#include <string>
#include <chrono>
#include <algorithm>

class ControlActionCalc : public rclcpp::Node
{
public:
    ControlActionCalc() : Node("control_action_calc")
    {
        RCLCPP_INFO(this->get_logger(), "Control Action Calculation Node has been started");

        control_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("/control_output", 10);

        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "local_path",
            rclcpp::QoS(10),
            std::bind(&ControlActionCalc::path_callback, this, std::placeholders::_1)
        );

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ControlActionCalc::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        const auto &poses = msg->poses;
        if (!current_odom_ || poses.empty()) {
            if (!current_odom_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for odometry data...");
            }
            // Not enough info to calculate a turn, send neutral signal
            navis_msgs::msg::ControlOut control_msg;
            control_msg.buzzer_strength = 0;
            control_msg.speaker_wav_index = 0;
            control_publisher_->publish(control_msg);
            return;
        }

        // Use a static lookahead point from the path
        auto current_pose = current_odom_->pose.pose;
        geometry_msgs::msg::Point target_waypoint;

        if (poses.size() > 4) {
            target_waypoint = poses[4].pose.position;
        } else {
            target_waypoint = poses.back().pose.position;
        }

        // Calculate angle to the next waypoint from current pose
        double target_angle = atan2(target_waypoint.y - current_pose.position.y, target_waypoint.x - current_pose.position.x);

        // Get current yaw from odometry quaternion
        auto q = current_pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double current_yaw = atan2(siny_cosp, cosy_cosp);

        double delta = target_angle - current_yaw;

        // Normalize the angle to be between -PI and PI
        while (delta > M_PI) delta -= 2 * M_PI;
        while (delta < -M_PI) delta += 2 * M_PI;

        // --- Haptic Feedback Logic ---
        // Scale the angle error to the haptic output range [-255, 255]
        int haptic_output = static_cast<int>((delta / (M_PI / 2.0)) * 255.0);
        haptic_output = std::clamp(haptic_output, -255, 255);

        // --- Audio Cue Logic ---
        const double turn_threshold = M_PI / 4; // 45 degrees
        std::string cue = "";
        if (delta > turn_threshold) {
            cue = "left";
        } else if (delta < -turn_threshold) {
            cue = "right";
        }

        uint8_t wav_index = 0;
        if (!cue.empty()) {
            if (cue != last_cue_ || (this->now() - last_cue_time_).seconds() > 5.0) {
                const auto& wav_map = get_wav_map();
                if (wav_map.count(cue)) {
                    wav_index = wav_map.at(cue);
                    RCLCPP_INFO(this->get_logger(), "Published audio cue: %s", cue.c_str());
                    last_cue_ = cue;
                    last_cue_time_ = this->now();
                }
            }
        }

        // --- Publish Control Message ---
        auto control_msg = navis_msgs::msg::ControlOut();
        control_msg.buzzer_strength = haptic_output;
        control_msg.speaker_wav_index = wav_index;
        control_publisher_->publish(control_msg);

        RCLCPP_INFO(this->get_logger(), "Published haptic intensity: %d", haptic_output);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr control_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;

    // State for preventing audio cue spam
    std::string last_cue_ = "";
    rclcpp::Time last_cue_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlActionCalc>());
  rclcpp::shutdown();
  return 0;
}