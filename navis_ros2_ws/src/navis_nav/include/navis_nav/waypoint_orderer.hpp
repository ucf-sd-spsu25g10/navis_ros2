#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <vector>

#include "std_msgs/msg/bool.hpp"
#include "navis_msgs/msg/waypoints_list.hpp"
#include "navis_nav/grocery_locations.hpp"

class WaypointOrderer
{
public:
    WaypointOrderer();
    void get_unordered_list();
    std::vector<std::string> get_list();

private:
  std::vector<std::string> grocery_list;

  std::vector<int> get_list_uart();
  void print_cur_list(std::string comment);
};