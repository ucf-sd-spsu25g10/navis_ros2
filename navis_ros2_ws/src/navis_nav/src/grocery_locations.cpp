#include "navis_nav/grocery_locations.hpp"

const float AISLE_MIN_HEIGHT_CONT = 0.0;
const float AISLE_MAX_HEIGHT_CONT = 10.0;

// Define the hash table instance
std::unordered_map<std::string, Item> store_map = {

    // int index;
    // int aisle;
    // float disc_y;
    // float cont_x
    // float cont_y;
    // int shelf_height;
    // bool side_of_aisle; what side of aisle the item is on, left is true. left is in respect to looking at the aisle from the bottom (low y)

    {"item_milk",            {0,  1, 1.0, 7.5, 0.0, 1, false}},
    {"item_bread",           {1,  1, 2.0, 22.3, -2.1, 1, true}},
    {"item_eggs",            {2,  1, 3.0, 29.4, -4.3, 1, false}},
    {"item_chicken",         {3,  1, 4.0, 36.5, -6.7, 1, true}},
    {"item_banana",          {4,  2, 1.0, 3.6, -15.5, 1, false}},
    {"item_cereal",          {5,  2, 2.0, 19.1, -20.5, 1, true}},
    {"item_coffee",          {6,  2, 3.0, 33.2, -23.5, 1, false}},
    {"item_carrot",          {7,  3, 1.0, 1.25, -26.5, 1, true}},
    {"item_fish",            {8,  3, 2.0, 17.1, -32.0, 0, false}},
    {"item_cheese",          {9,  3, 3.0, 33.7, -37.0, 0, true}},

    {"aisle1_bottom",   {12, 1, AISLE_MIN_HEIGHT_CONT, 2.3, 0.1, 0, true}},
    {"aisle1_top",      {13, 1, AISLE_MAX_HEIGHT_CONT, 42.2, -9.2, 0, true}},
    {"aisle2_bottom",   {14, 2, AISLE_MIN_HEIGHT_CONT, -0.4, -13.8, 0, true}},
    {"aisle2_top",      {15, 2, AISLE_MAX_HEIGHT_CONT, 39.6, -24.8, 0, true}},
    {"aisle3_bottom",   {16, 3, AISLE_MIN_HEIGHT_CONT, -1.3, -25.3, 0, true}},
    {"aisle3_top",      {17, 3, AISLE_MAX_HEIGHT_CONT, 36.2, -37.4, 0, true}},

    // {"item_milk",            {0,  1, 1.0, 6.4, -0.3, 1, false}},
    // {"item_bread",           {1,  1, 2.0, 16.7, -1.6, 1, true}},
    // {"item_eggs",            {2,  1, 3.0, 27.0, -3.9, 1, false}},
    // {"item_chicken",         {3,  2, 1.0, 6.4, -17.1, 1, true}},
    // {"item_banana",          {4,  2, 2.0, 18.2, -19.7, 1, false}},
    // {"item_cereal",          {5,  2, 3.0, 24.6, -20.7, 1, true}},
    // {"item_coffee",          {6,  3, 1.0, -8.5, -29.8, 1, false}},
    // {"item_carrot",          {7,  3, 2.0, 20.4, -36.9, 1, true}},
    // {"item_fish",            {8,  4, 1.0, -13.1, -40.0, 0, false}},
    // {"item_cheese",          {9,  4, 2.0, 17.4, -48.2, 0, true}},

    // {"aisle1_bottom",   {12, 1, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    // {"aisle1_top",      {13, 1, AISLE_MAX_HEIGHT_CONT, 34.4, -6.5, 0, true}},
    // {"aisle2_bottom",   {14, 2, AISLE_MIN_HEIGHT_CONT, -9.0, -12.5, 0, true}},
    // {"aisle2_top",      {15, 2, AISLE_MAX_HEIGHT_CONT, 29.3, -21.3, 0, true}},
    // {"aisle3_bottom",   {16, 3, AISLE_MIN_HEIGHT_CONT, -13.5, -27.9, 0, true}},
    // {"aisle3_top",      {17, 3, AISLE_MAX_HEIGHT_CONT, 24.5, -37.6, 0, true}},
    // {"aisle4_bottom",   {18, 4, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    // {"aisle4_top",      {19, 4, AISLE_MAX_HEIGHT_CONT, 20.9, -49.4, 0, true}},
    
};

// i know this looks fucking stupid having a map to map the keys of another map, but i had to change the keys for the store_map above to ints the night before demo and this is the quickest fix without having to change my waypoint ordering and management scripts based on key name parsing
std::unordered_map<int, std::string> index_to_item_name_map = {
    {0, "item_milk"},
    {1, "item_bread"},
    {2, "item_eggs"},
    {3, "item_chicken"},
    {4, "item_banana"},
    {5, "item_cereal"},
    {6, "item_coffee"},
    {7, "item_carrot"},
    {8, "item_fish"},
    {9, "item_cheese"},
    {10, "start_pose"},
    {11, "final_pose"},
    {12, "aisle1_bottom"},
    {13, "aisle1_top"},
    {14, "aisle2_bottom"},
    {15, "aisle2_top"},
    {16, "aisle3_bottom"},
    {17, "aisle3_top"},
    {18, "aisle4_bottom"},
    {19, "aisle4_top"},
 
};


// old map waypoints
    // {"milk",            {0,  1, 1.0, 16.0, 39.4, 1, false}},
    // {"bread",           {1,  1, 2.0, 26.0, 38.3, 1, true}},
    // {"eggs",            {2,  1, 3.0, 37.0, 37.8, 1, false}},
    // {"chicken",         {3,  2, 1.0, 10.0, 26.3, 1, true}},
    // {"banana",          {4,  2, 2.0, 29.0, 27.4, 1, false}},
    // {"cereal",          {5,  3, 1.0, 13.0, 9.2, 1, true}},
    // {"coffee",          {6,  3, 2.0, 23.0, 10.5, 1, false}},
    // {"carrot",          {7,  3, 3.0, 29.0, 11.5, 1, true}},
    // {"fish",            {8,  4, 1.0, 10.0, 0.6, 0, false}},
    // {"cheese",          {9,  4, 2.0, 25.0, 0.8, 0, true}},

    // {"start_pose",      {10, -1, 0.0, 10.0, 40.0, 0, true}},
    // {"final_pose",      {11, -1, 0.0, 3.0, 32.5, 0, true}},

    // {"aisle1_bottom",   {12, 1, AISLE_MIN_HEIGHT_CONT, 10.0, 40.0, 0, true}},
    // {"aisle1_top",      {13, 1, AISLE_MAX_HEIGHT_CONT, 40.5, 37.3, 0, true}},
    // {"aisle2_bottom",   {14, 2, AISLE_MIN_HEIGHT_CONT, 2.5, 26.7, 0, true}},
    // {"aisle2_top",      {15, 2, AISLE_MAX_HEIGHT_CONT, 38.4, 27.0, 0, true}},
    // {"aisle3_bottom",   {16, 3, AISLE_MIN_HEIGHT_CONT, 0.2, 11.0, 0, true}},
    // {"aisle3_top",      {17, 3, AISLE_MAX_HEIGHT_CONT, 37.4, 11.5, 0, true}},
    // {"aisle4_bottom",   {18, 4, AISLE_MIN_HEIGHT_CONT, 0.0, 0.0, 0, true}},
    // {"aisle4_top",      {19, 4, AISLE_MAX_HEIGHT_CONT, 37.5, 0.0, 0, true}},