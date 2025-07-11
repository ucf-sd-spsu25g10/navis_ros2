#include "navis_nav/grocery_locations.hpp"

const float AISLE_MIN_HEIGHT_CONT = 0.0;
const float AISLE_MAX_HEIGHT_CONT = 10.0;

// Define the hash table instance
std::unordered_map<std::string, Item> store_map = {

    // std::string name;
    // int aisle;
    // float disc_y;
    // float cont_x
    // float cont_y;
    // int shelf_height;
    // bool side_of_aisle; what side of aisle the item is on, left is true. left is in respect to looking at the aisle from the bottom (low y)

    {"start_pose", {"start_pose", -1, 0.0, 0.5, 0.0, 0, true}},
    {"final_pose", {"final_pose", -2, 0.0, 0.5, 0.0, 0, true}},

    {"item_a", {"a", 1, 4.1, 0.3, 7.0, 1, true}},
    {"item_b", {"a", 2, 6.7, 1.2, 7.0, 1, false}},
    {"item_c", {"a", 3, 3.4, 2.25, 7.0, 1, true}},

    {"item_d", {"a", 3, 1.3, 0.0, -6.0, 1, true}},
    {"item_e", {"a", 5, 7.8, -2.0, -2.0, 1, false}},
    {"item_f", {"a", 5, 8.5, -2.0, -6.0, 1, true}},
    
    {"aisle1_bottom", {"a", 1, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle1_top",    {"a", 1, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle2_bottom", {"a", 2, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle2_top",    {"a", 2, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle3_bottom", {"a", 3, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle3_top",    {"a", 3, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle4_bottom", {"a", 4, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle4_top",    {"a", 4, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle5_bottom", {"a", 5, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle5_top",    {"a", 5, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle6_bottom", {"a", 6, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle6_top",    {"a", 6, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle7_bottom", {"a", 7, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle7_top",    {"a", 7, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle8_bottom", {"a", 8, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle8_top",    {"a", 8, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle9_bottom", {"a", 9, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    {"aisle9_top",    {"a", 9, AISLE_MAX_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    
};