#ifndef WAREHOUSE_MAP_HPP
#define WAREHOUSE_MAP_HPP

#include <string>
#include <unordered_map>

struct Item {
    std::string name;
    int aisle;    // waypoint ordering
    int height;
    float cont_x; // For continuous routing
    float cont_y;
    int shelf_height;
};

extern std::unordered_map<std::string, Item> store_map;

#endif
