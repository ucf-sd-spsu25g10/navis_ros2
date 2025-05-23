#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navis_msgs/msg/waypoints_list.hpp"
#include "navis_msgs/msg/control_out.hpp"
#include "navis_nav/grocery_locations.hpp"

class WaypointManager : public rclcpp::Node
{
public:

    WaypointManager() : Node("waypoint_manager")
    {
        RCLCPP_INFO(this->get_logger(), "Talker C++ node has been created");

        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("rtabmap/goal", 10);
        speaker_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("control_output", 10);

        list_subscriber_ = this->create_subscription<navis_msgs::msg::WaypointsList>(
            "waypoint_list",
            rclcpp::QoS(10),
            std::bind(&WaypointManager::list_callback, this, std::placeholders::_1)
        );

        goal_reached_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "rtabmap/goal_reached",
            rclcpp::QoS(10),
            std::bind(&WaypointManager::goal_reached_callback, this, std::placeholders::_1)
        );

        cur_waypoint_idx = 0;
        number_of_waypoints = 0;

    }

private:

    rclcpp::Subscription<navis_msgs::msg::WaypointsList>::SharedPtr list_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr speaker_publisher_;

    std::vector<std::string> waypoint_list;
    int cur_waypoint_idx, number_of_waypoints;

    void list_callback(const navis_msgs::msg::WaypointsList::SharedPtr msg) {
        for (std::string& waypoint : msg->waypoints) {
            waypoint_list.push_back(waypoint);
        }

        number_of_waypoints = waypoint_list.size();
        RCLCPP_INFO(this->get_logger(), "Ordered waypoint list received, beginning navigation");
    }

    void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        
        if (msg->data) {

            RCLCPP_INFO(this->get_logger(), "Waypoint %d Reached", cur_waypoint_idx);

            // Spin til we have grocery list
            while (number_of_waypoints == 0) {}

            // TODO
            // Add waypoint reached logic for speaker out, eg shelf height, left or right, etc
            navis_msgs::msg::ControlOut control_msg;
            control_msg.buzzer_strength = 0;
            control_msg.speaker_wav_index = 0; // Add logic for this
            speaker_publisher_->publish(control_msg);

            // Process the message to be published
            geometry_msgs::msg::PoseStamped next_goal_msg;
            next_goal_msg.header.frame_id = "map";
            next_goal_msg.header.stamp = this->now();
            next_goal_msg.pose.position.x = store_map[waypoint_list[cur_waypoint_idx]].cont_x;  // your logic here
            next_goal_msg.pose.position.y = store_map[waypoint_list[cur_waypoint_idx]].cont_y;
            next_goal_msg.pose.orientation.w = 1.0;

            waypoint_publisher_->publish(next_goal_msg);

            ++cur_waypoint_idx;

            // TODO
            // Add final waypoint reached logic
            if (cur_waypoint_idx + 1 == number_of_waypoints) {
                RCLCPP_INFO(this->get_logger(), "Final Waypoint Reached", cur_waypoint_idx);
            }

        }
    }


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}
