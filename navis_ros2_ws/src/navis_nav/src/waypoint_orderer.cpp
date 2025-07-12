#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <algorithm>

#include "navis_nav/grocery_locations.hpp"
#include "navis_nav/waypoint_orderer.hpp"

WaypointOrderer::WaypointOrderer() {}

void WaypointOrderer::get_unordered_list() {
  std::vector<int> uart_list = get_list_uart();
  grocery_list.clear();
  
  for (int item_num : uart_list) {
    grocery_list.push_back("item_" + std::to_string(item_num));
  }
}

std::vector<std::string> WaypointOrderer::get_list() {

  // Wait until a list is received (not in a ros2 topic)
  // get_unordered_list();
  grocery_list = {
                  "item_a",
                  "item_b",
                  "item_c",
                  
                  // "item_f",
                  // "item_d",
                  // "item_e",
                  };

  int cur_aisle = store_map[grocery_list[0]].aisle;
  grocery_list.insert(grocery_list.begin(), "aisle" + std::to_string(cur_aisle) + "_top");
  grocery_list.insert(grocery_list.begin(), "aisle" + std::to_string(cur_aisle) + "_bottom");

  for (size_t i = 1; i < grocery_list.size(); i++) {
    std::string& str = grocery_list[i];

    if (str.find("aisle") != std::string::npos) continue;

    int temp_aisle = store_map[str].aisle;

    bool exists = std::any_of(grocery_list.begin(), grocery_list.end(),
                        [&](const std::string& s) {
                          return s.find("aisle" + std::to_string(temp_aisle)) != std::string::npos;
                        });

    if (temp_aisle != cur_aisle && !exists) {
      grocery_list.insert(grocery_list.begin() + i, "aisle" + std::to_string(temp_aisle) + "_top");
      ++i;

      grocery_list.insert(grocery_list.begin() + i, "aisle" + std::to_string(temp_aisle) + "_bottom");
      ++i;
      cur_aisle = temp_aisle;
    }
  }

  // print_cur_list("preset list");

  // Order list in terms of aisle
  std::sort(grocery_list.begin(), grocery_list.end(), [](std::string& a, std::string& b) {
    return store_map[a].aisle < store_map[b].aisle;
  });

  // print_cur_list("aisles sorted");

  // Order sublists alternating from down->top to top->down
  int start_window = 0, end_window = 0;
  bool ascending = true;
  cur_aisle = store_map[grocery_list[0]].aisle;

  for (std::string& str : grocery_list) {

    if (store_map[str].aisle != cur_aisle) {

      std::sort(
        grocery_list.begin() + start_window, 
        grocery_list.begin() + end_window, 
        [&](std::string& a, std::string& b) {
          return ascending ? (store_map[a].disc_y < store_map[b].disc_y) : (store_map[a].disc_y > store_map[b].disc_y);
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
      return ascending ? (store_map[a].disc_y < store_map[b].disc_y) : (store_map[a].disc_y > store_map[b].disc_y);
  });

  
  grocery_list.insert(grocery_list.begin(), "start_pose");
  int last_aisle = store_map[grocery_list[grocery_list.size()-1]].aisle;
  grocery_list.pop_back();
  grocery_list.insert(grocery_list.begin() + grocery_list.size(), "aisle" + std::to_string(last_aisle) + "_bottom");
  
  // print_cur_list("heights sorted");

  return grocery_list;
}

std::vector<int> WaypointOrderer::get_list_uart() {
  std::vector<int> numbers;
  const char* uart_dev = "/dev/ttyAMA1";  // RPi5 UART port
  
  int uart_fd = open(uart_dev, O_RDONLY | O_NOCTTY);
  if (uart_fd < 0) {
    std::cerr << "Failed to open UART" << std::endl;
    return numbers;
  }

  // Configure UART
  struct termios uart_config;
  tcgetattr(uart_fd, &uart_config);
  cfsetispeed(&uart_config, B115200);
  cfmakeraw(&uart_config);
  tcsetattr(uart_fd, TCSANOW, &uart_config);

  std::string buffer;
  char c;
  while (true) {
    if (read(uart_fd, &c, 1) > 0) {
      if (c == ';') break;
      if (c == ',') {
        if (!buffer.empty()) {
          numbers.push_back(std::stoi(buffer));
          buffer.clear();
        }
      } else if (isdigit(c)) {
        buffer += c;
      }
    }
  }
  
  // Add last number if exists
  if (!buffer.empty()) {
    numbers.push_back(std::stoi(buffer));
  }

  close(uart_fd);
  return numbers;
}

void WaypointOrderer::print_cur_list(std::string comment) {
  std::cout << "----------------------------" << std::endl;
  std::cout << comment << std::endl;
  std::cout << "item, aisle, height" << std::endl;
  for (std::string& str : grocery_list) {
    std::cout << str << ", " << store_map[str].aisle << ", " << store_map[str].disc_y << std::endl;
  }
  std::cout << "----------------------------\n" << std::endl;
}