#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "navis_msgs/msg/control_out.hpp"
#include "navis_nav/audio_mappings.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class AudioControlNode : public rclcpp::Node {
public:
    AudioControlNode() : Node("audio_control_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&AudioControlNode::odom_callback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan",
            10,
            std::bind(&AudioControlNode::path_callback, this, std::placeholders::_1));

        control_pub_ = this->create_publisher<navis_msgs::msg::ControlOut>("/control_output", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // Check every 500ms
            std::bind(&AudioControlNode::control_loop, this));
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
            return; // No data, do nothing
        }

        auto current_position = current_pose_->position;
        tf2::Quaternion q(
            current_pose_->orientation.x,
            current_pose_->orientation.y,
            current_pose_->orientation.z,
            current_pose_->orientation.w);
        tf2::Matrix3x3 m(q);
        double current_roll, current_pitch, current_yaw;
        m.getRPY(current_roll, current_pitch, current_yaw);

        geometry_msgs::msg::Point target_position;
        if (current_path_->poses.size() > 1) {
            target_position = current_path_->poses[1].pose.position;
        } else {
            target_position = current_path_->poses[0].pose.position;
        }

        double target_angle = atan2(
            target_position.y - current_position.y,
            target_position.x - current_position.x);

        double angle_error = target_angle - current_yaw;
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        // --- Audio Cue Logic ---
        const double turn_threshold = M_PI / 4; // 45 degrees

        if (angle_error > turn_threshold) {
            publish_audio_cue("left");
        } else if (angle_error < -turn_threshold) {
            publish_audio_cue("right");
        }
        // No audio cue if heading is generally correct
    }

    void publish_audio_cue(const std::string& cue) {
        // To prevent spamming cues, we can add a check here
        // to see if we've recently sent the same cue.
        if (cue == last_cue_ && (this->now() - last_cue_time_).seconds() < 5.0) {
            return; // Don't send the same cue within 5 seconds
        }

        auto control_msg = navis_msgs::msg::ControlOut();
        control_msg.buzzer_strength = 0; // Not used for audio
        if (wav_map.count(cue)) {
            control_msg.speaker_wav_index = wav_map[cue];
            control_pub_->publish(control_msg);
            RCLCPP_INFO(this->get_logger(), "Published audio cue: %s", cue.c_str());
            last_cue_ = cue;
            last_cue_time_ = this->now();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<geometry_msgs::msg::Pose> current_pose_;
    nav_msgs::msg::Path::SharedPtr current_path_;

    // State for preventing cue spam
    std::string last_cue_ = "";
    rclcpp::Time last_cue_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioControlNode>());
    rclcpp::shutdown();
    return 0;
}
