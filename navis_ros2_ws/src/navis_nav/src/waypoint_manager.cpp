#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navis_msgs/msg/waypoints_list.hpp"
#include "navis_msgs/msg/control_out.hpp"
#include "navis_nav/grocery_locations.hpp"
#include "navis_nav/waypoint_orderer.hpp"

class WaypointManager : public rclcpp::Node
{
public:

    WaypointManager() : Node("waypoint_manager")
    {
        RCLCPP_INFO(this->get_logger(), "Talker C++ node has been created");

        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("rtabmap/goal", 10);
        speaker_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("control_output", 10);

        goal_reached_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "rtabmap/goal_reached",
            rclcpp::QoS(10),
            std::bind(&WaypointManager::goal_reached_callback, this, std::placeholders::_1)
        );

        cur_waypoint_idx = 0;
        number_of_waypoints = 0;

        get_list();
    }

private:

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr speaker_publisher_;

    std::vector<std::string> waypoint_list;
    int cur_waypoint_idx, number_of_waypoints;

    WaypointOrderer waypoint_orderer;

    std::unordered_map<std::string, int> wav_map = {
        {"left", 0},            // Turn left 90 degrees (when reach bottom of aisle)
        {"right", 1},           // Turn right 90 degrees (when reach top of aisle)
        {"turnaround", 2},      // Do a 180
        {"final", 3},           // Final Destination reached
        {"shelf_1", 4},         // Item on shelf #
        {"shelf_2", 5},
        {"shelf_3", 6},
        {"item_l", 7},          // Item on left / right
        {"item_r", 8},
        {"straight_1",  9},     // Go straight # meters
        {"straight_2",  10},
        {"straight_3",  11},
        {"straight_4",  12},
        {"straight_5",  13},
        {"straight_6",  14},
        {"straight_7",  15},
        {"straight_8",  16},
        {"straight_9",  17},
        {"straight_10", 18},
    };

    int sleep_time_ms = 500;

    // true = x, false = y
    void add_go_straight_cmd(bool x_or_y) {
        navis_msgs::msg::ControlOut control_msg;
        control_msg.buzzer_strength = 0;
        int distance_to_next = 0;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));

        if (x_or_y) {
            distance_to_next = std::min(10.0f, 
                               std::round(std::abs(store_map[waypoint_list[cur_waypoint_idx+1]].cont_y - 
                                                   store_map[waypoint_list[cur_waypoint_idx]].cont_y)));
        } else {
            distance_to_next = std::min(10.0f, 
                               std::round(std::abs(store_map[waypoint_list[cur_waypoint_idx+1]].cont_y - 
                                                   store_map[waypoint_list[cur_waypoint_idx]].cont_y)));
        }

        control_msg.speaker_wav_index = wav_map["straight_" + std::to_string(distance_to_next)];
        speaker_publisher_->publish(control_msg);
        RCLCPP_INFO(this->get_logger(), "Indicating go straight %d meters", distance_to_next);
    }

    void get_list() {
        waypoint_orderer.get_unordered_list();
        waypoint_list = waypoint_orderer.order_list();
        number_of_waypoints = waypoint_list.size();
        RCLCPP_INFO(this->get_logger(), "Ordered waypoint list received, beginning navigation");

        // TODO Indiciating to ensure we are at the start location
    }

    void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        
        if (msg->data) {

            navis_msgs::msg::ControlOut control_msg;
            control_msg.buzzer_strength = 0;
            std::string cur_waypoint_str = waypoint_list[cur_waypoint_idx];

            RCLCPP_INFO(this->get_logger(), "Waypoint %d Reached", cur_waypoint_idx);

            // Spin til we have grocery list
            while (number_of_waypoints == 0) {}

            // Final Waypoint
            if (cur_waypoint_idx + 1 == number_of_waypoints) {
                control_msg.speaker_wav_index = wav_map["final"];
                speaker_publisher_->publish(control_msg);
                RCLCPP_INFO(this->get_logger(), "Final Waypoint Reached, %s", cur_waypoint_str.c_str());
            }

            // Navigate to Final Waypoint
            if (cur_waypoint_idx + 2 == number_of_waypoints) {
                control_msg.speaker_wav_index = wav_map["final"];
                speaker_publisher_->publish(control_msg);
                RCLCPP_INFO(this->get_logger(), "Navigating to Final Waypoint, %s", cur_waypoint_str.c_str());
                add_go_straight_cmd(true);
            }

            // Final Item
            if (cur_waypoint_idx + 3 == number_of_waypoints) {
                control_msg.speaker_wav_index = wav_map["turnaround"];
                speaker_publisher_->publish(control_msg);
                RCLCPP_INFO(this->get_logger(), "Final Item Obtained, %s", cur_waypoint_str.c_str());
                add_go_straight_cmd(false);
            }

            // Item 
            if (cur_waypoint_str.find("item") != std::string::npos) {
                control_msg.speaker_wav_index = wav_map["shelf_" + store_map[cur_waypoint_str].shelf_height];
                speaker_publisher_->publish(control_msg);
                RCLCPP_INFO(this->get_logger(), "Item %s Reached, Indicating shelf %d", cur_waypoint_str.c_str(), store_map[cur_waypoint_str].shelf_height);
                add_go_straight_cmd(false);
            }

            // End of Aisle
            if (cur_waypoint_str.find("aisle") != std::string::npos) {
                control_msg.speaker_wav_index = (cur_waypoint_str.find("bottom") != std::string::npos) ? wav_map["left"] : wav_map["right"];
                speaker_publisher_->publish(control_msg);
                RCLCPP_INFO(this->get_logger(), "End of Aisle Reached, %s", cur_waypoint_str.c_str());
                add_go_straight_cmd(true);
            }
            
            cur_waypoint_idx++;

            // Process the message to be published
            geometry_msgs::msg::PoseStamped next_goal_msg;
            next_goal_msg.header.frame_id = "map";
            next_goal_msg.header.stamp = this->now();
            next_goal_msg.pose.position.x = aisle_x_disc2cont_map[store_map[waypoint_list[cur_waypoint_idx]].aisle];  // your logic here
            next_goal_msg.pose.position.y = store_map[waypoint_list[cur_waypoint_idx]].cont_y;
            next_goal_msg.pose.orientation.w = 1.0;

            waypoint_publisher_->publish(next_goal_msg);

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
