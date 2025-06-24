#ifndef WAREHOUSE_MAP_HPP
#define WAREHOUSE_MAP_HPP

#include <string>
#include <unordered_map>

extern const float AISLE_MIN_HEIGHT_CONT;
extern const float AISLE_MAX_HEIGHT_CONT;

struct Item {
    std::string name;
    int aisle;    // waypoint ordering
    float cont_y;
    int shelf_height;
};

extern std::unordered_map<std::string, Item> store_map;
extern std::unordered_map<int, float> aisle_x_disc2cont_map;

#endif
