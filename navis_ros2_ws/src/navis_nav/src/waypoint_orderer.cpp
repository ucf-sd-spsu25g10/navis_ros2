#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "navis_msgs/msg/waypoints_list.hpp"
#include "navis_nav/grocery_locations.hpp"

class WaypointOrderer : public rclcpp::Node
{
public:
  WaypointOrderer() : Node("waypoint_orderer")
  {
    RCLCPP_INFO(this->get_logger(), "Talker C++ node has been created");

    // Setup up Publisher to whatever list topic
    list_publisher_ = this->create_publisher<navis_msgs::msg::WaypointsList>("waypoints_topic", 10);

    // Wait until a list is received (not in a ros2 topic)
    // grocery_list = get_unordered_list();
    grocery_list = {"item_f",
                    "item_b",
                    "item_a",
                    "item_d",
                    "item_c",
                    "item_e",};

    publish_ordered_list();
  }

private:
  rclcpp::Publisher<navis_msgs::msg::WaypointsList>::SharedPtr list_publisher_;
  std::vector<std::string> grocery_list;

  // TODO
  void get_unordered_list() {
    return;
  }

  void publish_ordered_list() {
    
    // Vector of strings that = keys in store_map hash table
    auto msg = navis_msgs::msg::WaypointsList();

    print_cur_list("preset list");

    // Order list in terms of aisle
    std::sort(grocery_list.begin(), grocery_list.end(), [](std::string& a, std::string& b) {
      return store_map[a].aisle < store_map[b].aisle;
    });

    print_cur_list("aisles sorted");

    // Order sublists alternating from down->top to top->down
    int start_window = 0, end_window = 0;
    int cur_aisle = store_map[grocery_list[0]].aisle;
    bool ascending = true;

    for (std::string& str : grocery_list) {

      if (store_map[str].aisle != cur_aisle) {

        std::sort(
          grocery_list.begin() + start_window, 
          grocery_list.begin() + end_window, 
          [&](std::string& a, std::string& b) {
            return ascending ? (store_map[a].height < store_map[b].height) : (store_map[a].height > store_map[b].height);
        });

        cur_aisle = store_map[str].aisle;
        start_window = end_window;
        ascending = !ascending;
      }

      ++end_window;
    }

    std::sort(
      grocery_list.begin() + start_window, 
      grocery_list.begin() + end_window, 
      [&](std::string& a, std::string& b) {
        return ascending ? (store_map[a].height < store_map[b].height) : (store_map[a].height > store_map[b].height);
    });

    print_cur_list("heights sorted");
    
    // put in ros2 message format
    for (std::string& str : grocery_list) {
      msg.waypoints.push_back(str);
    }

    list_publisher_->publish(msg);
    return;
  }

  void print_cur_list(std::string comment) {
    std::cout << "----------------------------" << std::endl;
    std::cout << comment << std::endl;
    std::cout << "item, aisle, height" << std::endl;
    for (std::string& str : grocery_list) {
      std::cout << str << ", " << store_map[str].aisle << ", " << store_map[str].height << std::endl;
    }
    std::cout << "----------------------------\n" << std::endl;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointOrderer>());
  rclcpp::shutdown();
  return 0;
}
