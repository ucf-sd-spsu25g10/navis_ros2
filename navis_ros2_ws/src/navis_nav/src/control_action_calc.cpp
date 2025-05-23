#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "nav_msgs/msg/path.hpp"
#include "navis_msgs/msg/control_out.hpp"

class ControlActionCalc : public rclcpp::Node
{
public:

    ControlActionCalc() : Node("control_action_calc")
    {
        RCLCPP_INFO(this->get_logger(), "Control Action Calculation Node has been created");

        control_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("control_output", 10);

        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            // "/rtabmab/global_path",
            "/rtabmap/local_path",
            rclcpp::QoS(10),
            std::bind(&ControlActionCalc::path_callback, this, std::placeholders::_1)
        );

    }

private:

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;

    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr control_publisher_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        const auto &poses = msg->poses;
        if (poses.size() < 3) return;

        for (size_t i = 1; i < poses.size() - 1; ++i) {
            auto p1 = poses[i-1].pose.position;
            auto p2 = poses[i].pose.position;
            auto p3 = poses[i+1].pose.position;

            double angle1 = atan2(p2.y - p1.y, p2.x - p1.x);
            double angle2 = atan2(p3.y - p2.y, p3.x - p2.x);
            double delta = angle2 - angle1;

            while (delta > M_PI) delta -= 2 * M_PI;
            while (delta < -M_PI) delta += 2 * M_PI;

            int intensity = static_cast<int>((delta / (M_PI / 2.0)) * 256.0);
            intensity = std::clamp(intensity, -256, 256);

            navis_msgs::msg::ControlOut control_msg;
            control_msg.buzzer_strength = intensity;
            control_msg.speaker_wav_index = 0;
            control_publisher_->publish(control_msg);

            RCLCPP_INFO(this->get_logger(), "Published turn intensity: %d", intensity);
        }
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlActionCalc>());
  rclcpp::shutdown();
  return 0;
}
