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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "navis_msgs/msg/waypoints_list.hpp"
#include "navis_msgs/msg/control_out.hpp"
#include "navis_nav/grocery_locations.hpp"
#include "navis_nav/waypoint_orderer.hpp"

class WaypointManager : public rclcpp::Node
{
public:

    WaypointManager() : Node("waypoint_manager"), wav_map_(get_wav_map())
    {
        RCLCPP_INFO(this->get_logger(), "WaypointManager node has been created");

        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
        speaker_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("/control_output", 10);
        buzzer_publisher_ = this->create_publisher<navis_msgs::msg::ControlOut>("/control_output", 10);

        localization_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization_pose",
            rclcpp::QoS(10),
            std::bind(&WaypointManager::pose_callback, this, std::placeholders::_1)
        );

        cur_waypoint_idx = -1;
        number_of_waypoints = 0;
        got_list = false;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // delay
            [this]() {
                get_list();
            }
        );
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;

    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr speaker_publisher_;
    rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr buzzer_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> waypoint_list;
    int cur_waypoint_idx, number_of_waypoints;

    WaypointOrderer waypoint_orderer;
    bool got_list;

    std::condition_variable cv_;
    float goal_threshold = 0.5; // meters

    #include "navis_nav/audio_mappings.hpp"

    int sleep_time_ms = 250;

    const std::unordered_map<std::string, int>& wav_map_;

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
        control_msg.speaker_wav_index = wav_map_.at("straight");
        speaker_publisher_->publish(control_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map_.at(std::to_string(distance_to_next));
        speaker_publisher_->publish(control_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        control_msg.speaker_wav_index = wav_map_.at("meters");
        speaker_publisher_->publish(control_msg);

        RCLCPP_INFO(this->get_logger(), "Indicating go straight %d meters", distance_to_next);
    }

    void get_list() {
        if (!got_list) {
            waypoint_list = waypoint_orderer.get_list();
            got_list = true;
    
            for (const std::string& str : waypoint_list) {
                RCLCPP_INFO(this->get_logger(), "%s, %d, %.2f", 
                            str.c_str(), 
                            store_map[str].aisle, 
                            store_map[str].disc_y);
            }
    
            number_of_waypoints = waypoint_list.size();
            RCLCPP_INFO(this->get_logger(), "Ordered waypoint list received, beginning navigation to starting point. %d total waypoints", number_of_waypoints);

            process_waypoint_logic();
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        
        timer_->cancel();
        
        auto loc_x = msg->pose.pose.position.x;
        auto loc_y = msg->pose.pose.position.y;

        auto goal_x = store_map[waypoint_list[cur_waypoint_idx]].cont_x;
        auto goal_y = store_map[waypoint_list[cur_waypoint_idx]].cont_y;

        float distance_to_goal = std::sqrt(std::pow(goal_x - loc_x, 2) + std::pow(goal_y - loc_y, 2));

        if (distance_to_goal < goal_threshold) {
            process_waypoint_logic();
            RCLCPP_INFO(this->get_logger(), "Waypoint reached. Navigating to next waypoint.");
        } else {
            // Get absolute angle of line from current location to next waypoint

            // --- Current yaw from quaternion ---
            auto qx = msg->pose.pose.orientation.x;
            auto qy = msg->pose.pose.orientation.y;
            auto qz = msg->pose.pose.orientation.z;
            auto qw = msg->pose.pose.orientation.w;

            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            double current_yaw = std::atan2(siny_cosp, cosy_cosp);

            // --- Desired yaw to goal ---
            double desired_yaw = std::atan2(goal_y - loc_y, goal_x - loc_x);

            // --- Heading error ---
            double heading_error = desired_yaw - current_yaw;

            // Normalize to [-π, π]
            while (heading_error > M_PI)
                heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI)
                heading_error += 2.0 * M_PI;

            // --- Map to [-255, 255] ---
            double scale = heading_error / (M_PI / 2.0);  // Normalize: ±π/2 → ±1
            scale = std::max(std::min(scale, 1.0), -1.0); // Clip

            int orientation_instruction_ = static_cast<int>(scale * 255.0);

            // Optional: Print debug info
            RCLCPP_INFO(this->get_logger(), "CurYaw: %.2f deg, DesiredYaw: %.2f deg, Error: %.2f deg, Instruction: %d",
                        current_yaw * 180.0 / M_PI,
                        desired_yaw * 180.0 / M_PI,
                        heading_error * 180.0 / M_PI,
                        orientation_instruction_);
                        
            auto control_msg = navis_msgs::msg::ControlOut();
            control_msg.buzzer_strength = orientation_instruction_;
            control_msg.speaker_wav_index = -1;
            buzzer_publisher_->publish(control_msg);
        }
    }

    void process_waypoint_logic() {

        navis_msgs::msg::ControlOut control_msg;
        control_msg.buzzer_strength = 0;
        std::string cur_waypoint_str;
        if (cur_waypoint_idx == -1) {
            cur_waypoint_str == "";
        } else {
            cur_waypoint_str = waypoint_list[cur_waypoint_idx];
        }

        RCLCPP_INFO(this->get_logger(), "Processing Waypoint %d", cur_waypoint_idx);

        // Spin until we have grocery list
        while (number_of_waypoints == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // First Waypoint
        if (cur_waypoint_idx == -1) {
            RCLCPP_INFO(this->get_logger(), "Preprocessing starting waypoint");
        }

        // Final Waypoint
        else if (cur_waypoint_idx + 1 == number_of_waypoints) {
            control_msg.speaker_wav_index = wav_map_.at("final");
            speaker_publisher_->publish(control_msg);
            RCLCPP_INFO(this->get_logger(), "Final Waypoint Reached, %s", cur_waypoint_str.c_str());
        }

        // Item 
        else if (cur_waypoint_str.find("item") != std::string::npos) {
            
            // side_of_aisle -> true = left when looking at the aisle from the bottom
            // l_r = l_r if headed up, !l_r if headed down
            bool l_r = ((store_map[waypoint_list[cur_waypoint_idx]].disc_y - store_map[cur_waypoint_str].disc_y) >= 0 ) 
                        ?  store_map[cur_waypoint_str].side_of_aisle 
                        : !store_map[cur_waypoint_str].side_of_aisle;
            std::string l_r_str = (l_r) ? "left" : "right";

            control_msg.speaker_wav_index = wav_map_.at("item");
            speaker_publisher_->publish(control_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            control_msg.speaker_wav_index = wav_map_.at(l_r_str);
            speaker_publisher_->publish(control_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            control_msg.speaker_wav_index = wav_map_.at("shelf");
            speaker_publisher_->publish(control_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            control_msg.speaker_wav_index = wav_map_.at(std::to_string(store_map[cur_waypoint_str].shelf_height));
            speaker_publisher_->publish(control_msg);
            
            RCLCPP_INFO(this->get_logger(), "Item %s Reached, Indicating shelf %d, Indicating on side %s", cur_waypoint_str.c_str(), store_map[cur_waypoint_str].shelf_height, l_r_str.c_str());

            add_go_straight_cmd(false);

        }

        // End of Aisle
        else if (cur_waypoint_str.find("aisle") != std::string::npos) {
            std::string l_r = (cur_waypoint_str.find("bottom") != std::string::npos) ? "left" : "right";
            RCLCPP_INFO(this->get_logger(), "End of Aisle %s Reached, Indicating turn %s", cur_waypoint_str.c_str(), l_r.c_str());
            
            control_msg.speaker_wav_index = wav_map_.at("turn");
            speaker_publisher_->publish(control_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            control_msg.speaker_wav_index = wav_map_.at(l_r);
            speaker_publisher_->publish(control_msg);
            
            add_go_straight_cmd(true);
        }
        
        cur_waypoint_idx++;

        // Process the message to be published, this is purely for rviz visualization
        geometry_msgs::msg::PoseStamped next_goal_msg;
        next_goal_msg.header.frame_id = "map";
        next_goal_msg.header.stamp = this->now();

        next_goal_msg.pose.position.x = store_map[waypoint_list[cur_waypoint_idx]].cont_x;
        next_goal_msg.pose.position.y = store_map[waypoint_list[cur_waypoint_idx]].cont_y;
        next_goal_msg.pose.position.z = 0.0;
        next_goal_msg.pose.orientation.x = 0.0;
        next_goal_msg.pose.orientation.y = 0.0;
        next_goal_msg.pose.orientation.z = 0.0;
        next_goal_msg.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", 
            store_map[waypoint_list[cur_waypoint_idx]].cont_x,
            store_map[waypoint_list[cur_waypoint_idx]].cont_y);

    }


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}


