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
    grocery_list.push_back(index_to_item_name_map[item_num]);
  }
}

std::vector<std::string> WaypointOrderer::get_list() {

  // Wait until a list is received (not in a ros2 topic)
  get_unordered_list();
  // grocery_list = {
  //                 "milk",
  //                 "chicken",
  //                 "banana",
  //                 "cereal",
  //                 "coffee",
  //                 "fish"
  //                 };

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

  // print_cur_list("heights sorted");

  return grocery_list;
}

std::vector<int> WaypointOrderer::get_list_uart() {
  std::vector<int> numbers;
  const char* uart_dev = "/dev/ttyAMA0";  // RPi5 UART port
  
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
      buffer += c;
    }
  }
  close(uart_fd);

  // Trim whitespace
  buffer.erase(0, buffer.find_first_not_of(" \n\r\t"));
  buffer.erase(buffer.find_last_not_of(" \n\r\t") + 1);

  // Check for JSON array
  if (!buffer.empty() && buffer[0] == '[') {
    // Parse JSON array manually (no external library)
    std::string num;
    for (size_t i = 1; i < buffer.size(); ++i) {
      char ch = buffer[i];
      if (isdigit(ch)) {
        num += ch;
      } else if (ch == ',' || ch == ']') {
        if (!num.empty()) {
          numbers.push_back(std::stoi(num));
          num.clear();
        }
        if (ch == ']') break;
      }
    }
  } else {
    // Original comma-separated parsing
    std::string num;
    for (char ch : buffer) {
      if (isdigit(ch)) {
        num += ch;
      } else if (ch == ',') {
        if (!num.empty()) {
          numbers.push_back(std::stoi(num));
          num.clear();
        }
      }
    }
    if (!num.empty()) {
      numbers.push_back(std::stoi(num));
    }
  }

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