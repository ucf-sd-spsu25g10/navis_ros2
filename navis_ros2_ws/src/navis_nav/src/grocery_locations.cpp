#include "navis_nav/grocery_locations.hpp"

const float AISLE_MIN_HEIGHT_CONT = 0.0;
const float AISLE_MAX_HEIGHT_CONT = 10.0;

// Define the hash table instance
std::unordered_map<std::string, Item> store_map = {

    // std::string name;
    // int aisle;
    // float cont_y;
    // int shelf_height;

    {"start_pose", {"start_pose", -1, 0.0, 0}},
    {"final_pose", {"final_pose", -2, 0.0, 0}},

    {"item_a", {"a", 2, 3.4, 1}},
    {"item_b", {"a", 3, 6.7, 1}},
    {"item_c", {"a", 3, 4.1, 1}},
    {"item_d", {"a", 3, 1.3, 1}},
    {"item_e", {"a", 5, 7.8, 1}},
    {"item_f", {"a", 5, 8.5, 1}},
    
    {"aisle1_bottom", {"a", 1, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle1_top",    {"a", 1, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle2_bottom", {"a", 2, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle2_top",    {"a", 2, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle3_bottom", {"a", 3, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle3_top",    {"a", 3, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle4_bottom", {"a", 4, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle4_top",    {"a", 4, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle5_bottom", {"a", 5, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle5_top",    {"a", 5, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle6_bottom", {"a", 6, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle6_top",    {"a", 6, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle7_bottom", {"a", 7, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle7_top",    {"a", 7, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle8_bottom", {"a", 8, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle8_top",    {"a", 8, AISLE_MAX_HEIGHT_CONT, 0}},
    {"aisle9_bottom", {"a", 9, AISLE_MIN_HEIGHT_CONT, 0}},
    {"aisle9_top",    {"a", 9, AISLE_MAX_HEIGHT_CONT, 0}},
    
};

std::unordered_map<int, float> aisle_x_disc2cont_map {
    {-1, 0.0},
    {-2, 0.0},
    {1, 1.0},
    {2, 2.0},
    {3, 3.0},
    {4, 4.0},
    {5, 5.0},
    {6, 6.0},
    {7, 7.0},
    {8, 8.0},
    {9, 9.0},
};