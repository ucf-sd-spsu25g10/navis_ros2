#include "navis_nav/grocery_locations.hpp"

// Define the hash table instance
std::unordered_map<std::string, Item> store_map = {

    // std::string name;
    // int aisle;
    // int height;
    // float cont_x;
    // float cont_y;
    // int shelf_height;

    {"item_a", {"a", 2, 1, 0.0, 0.0, 1}},
    {"item_b", {"a", 3, 3, 0.0, 0.0, 1}},
    {"item_c", {"a", 3, 2, 0.0, 0.0, 1}},
    {"item_d", {"a", 3, 1, 0.0, 0.0, 1}},
    {"item_e", {"a", 5, 1, 0.0, 0.0, 1}},
    {"item_f", {"a", 5, 2, 0.0, 0.0, 1}},
};