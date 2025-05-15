#include "navis_nav/grocery_locations.hpp"

// Define the hash table instance
std::unordered_map<std::string, Item> store_map = {

    // int aisle;
    // int height;
    // float cont_x;
    // float cont_y;

    {"item_a", {2, 1, 0.0, 0.0}},
    {"item_b", {3, 3, 0.0, 0.0}},
    {"item_c", {3, 2, 0.0, 0.0}},
    {"item_d", {3, 1, 0.0, 0.0}},
    {"item_e", {5, 1, 0.0, 0.0}},
    {"item_f", {5, 2, 0.0, 0.0}},
};