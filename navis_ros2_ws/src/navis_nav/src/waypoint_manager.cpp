#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
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
        RCLCPP_INFO(this->get_logger(), "WaypointManager node has been created");

        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("rtabmap/goal", 10);
        speaker_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("control_output", 10);

        goal_reached_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "rtabmap/goal_reached",
            rclcpp::QoS(10),
            std::bind(&WaypointManager::ros_goal_reached_callback, this, std::placeholders::_1)
        );

        button_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gpio_controller/state",
            rclcpp::QoS(10),
            std::bind(&WaypointManager::button_callback, this, std::placeholders::_1)
        );

        cur_waypoint_idx = 0;
        number_of_waypoints = 0;

        get_list();
        RCLCPP_INFO(this->get_logger(), "Waiting for button press to start...");
    }

private:

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr button_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr speaker_publisher_;

    std::vector<std::string> waypoint_list;
    int cur_waypoint_idx, number_of_waypoints;

    WaypointOrderer waypoint_orderer;

    std::mutex goal_mutex_;
    std::condition_variable cv_;
    bool button_pressed_ = false;

    #include "navis_nav/audio_mappings.hpp"

    int sleep_time_ms = 500;

    // true = x, false = y
    void add_go_straight_cmd(bool x_or_y) {
        navis_msgs::msg::ControlOut control_msg;
        control_msg.buzzer_strength = 0;
        int distance_to_next = 0;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));

        if (x_or_y) {
            distance_to_next = std::min(10.0f, 
                               std::round(std::abs(store_map[waypoint_list[cur_waypoint_idx+1]].disc_y - 
                                                   store_map[waypoint_list[cur_waypoint_idx]].disc_y)));
        } else {
            distance_to_next = std::min(10.0f, 
                               std::round(std::abs(store_map[waypoint_list[cur_waypoint_idx+1]].disc_y - 
                                                   store_map[waypoint_list[cur_waypoint_idx]].disc_y)));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        const auto& wav_map = get_wav_map();
        control_msg.speaker_wav_index = wav_map.at("straight");
        speaker_publisher_->publish(control_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(std::to_string(distance_to_next));
        speaker_publisher_->publish(control_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at("meters");
        speaker_publisher_->publish(control_msg);

        RCLCPP_INFO(this->get_logger(), "Indicating go straight %d meters", distance_to_next);
    }

    void get_list() {
        waypoint_orderer.get_unordered_list();
        waypoint_list = waypoint_orderer.order_list();
        number_of_waypoints = waypoint_list.size();
        RCLCPP_INFO(this->get_logger(), "Ordered waypoint list received, beginning navigation");
    }

    void button_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (msg->data > 0.5) { // Button is pressed
            bool start_now = false;
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                if (cur_waypoint_idx > 0) {
                    RCLCPP_INFO(this->get_logger(), "Button pressed, continuing navigation.");
                    button_pressed_ = true;
                    cv_.notify_one();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Button pressed, starting navigation.");
                    start_now = true;
                }
            }

            if (start_now) {
                auto bool_msg = std::make_shared<std_msgs::msg::Bool>();
                bool_msg->data = true;
                process_waypoint_logic(bool_msg);
            }
        }
    }

    void ros_goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Waypoint reached. Waiting for button press to get next item.");

            std::unique_lock<std::mutex> lock(goal_mutex_);
            button_pressed_ = false; // Reset before waiting
            cv_.wait(lock, [this]{ return button_pressed_; });
            lock.unlock();

            // Debounce
            std::this_thread::sleep_for(std::chrono::seconds(1));

            process_waypoint_logic(msg);
        }
    }

    void process_waypoint_logic(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        
        if (msg->data) {

            navis_msgs::msg::ControlOut control_msg;
            control_msg.buzzer_strength = 0;
            std::string cur_waypoint_str = waypoint_list[cur_waypoint_idx];

            RCLCPP_INFO(this->get_logger(), "Processing Waypoint %d", cur_waypoint_idx);

            // Spin til we have grocery list
            while (number_of_waypoints == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // First Waypoint
            if (cur_waypoint_idx == 0) {
                RCLCPP_INFO(this->get_logger(), "First Waypoint Reached, Indicating to navigate to aisle %d with assistance", store_map[waypoint_list[cur_waypoint_idx+1]].aisle);
                
        control_msg.speaker_wav_index = wav_map.at("first");
                speaker_publisher_->publish(control_msg);
            
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at("aisle");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(std::to_string(store_map[cur_waypoint_str].aisle));
                speaker_publisher_->publish(control_msg);
            }

            // Final Waypoint
            else if (cur_waypoint_idx + 1 == number_of_waypoints) {
        control_msg.speaker_wav_index = wav_map.at("final");
                speaker_publisher_->publish(control_msg);
                RCLCPP_INFO(this->get_logger(), "Final Waypoint Reached, %s", cur_waypoint_str.c_str());
            }

            // Final Item
            else if (cur_waypoint_idx + 2 == number_of_waypoints) {

                // side_of_aisle -> true = left when looking at the aisle from the bottom
                // l_r = l_r if headed up, !l_r if headed down
                bool l_r = ((store_map[waypoint_list[cur_waypoint_idx]].disc_y - store_map[cur_waypoint_str].disc_y) >= 0 ) 
                            ?  store_map[cur_waypoint_str].side_of_aisle 
                            : !store_map[cur_waypoint_str].side_of_aisle;
                std::string l_r_str = (l_r) ? "left" : "right";

        control_msg.speaker_wav_index = wav_map.at("item");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(l_r_str);
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at("shelf");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(std::to_string(store_map[cur_waypoint_str].shelf_height));
                speaker_publisher_->publish(control_msg);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at("turn");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at("around");
                speaker_publisher_->publish(control_msg);
                
                add_go_straight_cmd(false);
                
                RCLCPP_INFO(this->get_logger(), "Final Item %s Reached, Indicating shelf %d, Indicating on side %s, Indicating turn around", cur_waypoint_str.c_str(), store_map[cur_waypoint_str].shelf_height, l_r_str.c_str());
            }

            // Item 
            else if (cur_waypoint_str.find("item") != std::string::npos) {
                
                // side_of_aisle -> true = left when looking at the aisle from the bottom
                // l_r = l_r if headed up, !l_r if headed down
                bool l_r = ((store_map[waypoint_list[cur_waypoint_idx]].disc_y - store_map[cur_waypoint_str].disc_y) >= 0 ) 
                            ?  store_map[cur_waypoint_str].side_of_aisle 
                            : !store_map[cur_waypoint_str].side_of_aisle;
                std::string l_r_str = (l_r) ? "left" : "right";

        control_msg.speaker_wav_index = wav_map.at("item");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(l_r_str);
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at("shelf");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(std::to_string(store_map[cur_waypoint_str].shelf_height));
                speaker_publisher_->publish(control_msg);
                
                RCLCPP_INFO(this->get_logger(), "Item %s Reached, Indicating shelf %d, Indicating on side %s", cur_waypoint_str.c_str(), store_map[cur_waypoint_str].shelf_height, l_r_str.c_str());

                add_go_straight_cmd(false);

            }

            // End of Aisle
            else if (cur_waypoint_str.find("aisle") != std::string::npos) {
                std::string l_r = (cur_waypoint_str.find("bottom") != std::string::npos) ? "left" : "right";
                RCLCPP_INFO(this->get_logger(), "End of Aisle %s Reached, Indicating turn %s", cur_waypoint_str.c_str(), l_r.c_str());
                
        control_msg.speaker_wav_index = wav_map.at("turn");
                speaker_publisher_->publish(control_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map.at(l_r);
                speaker_publisher_->publish(control_msg);
                
                add_go_straight_cmd(true);
            }
            
            cur_waypoint_idx++;

            // Process the message to be published
            geometry_msgs::msg::PoseStamped next_goal_msg;
            next_goal_msg.header.frame_id = "map";
            next_goal_msg.header.stamp = this->now();
            next_goal_msg.pose.position.x = store_map[waypoint_list[cur_waypoint_idx]].cont_x;  // your logic here
            next_goal_msg.pose.position.y = store_map[waypoint_list[cur_waypoint_idx]].cont_y;
            next_goal_msg.pose.orientation.w = 1.0;

            waypoint_publisher_->publish(next_goal_msg);
            RCLCPP_INFO(this->get_logger(), "\n");

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


