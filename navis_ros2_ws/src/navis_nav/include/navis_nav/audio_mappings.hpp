#pragma once

#include <unordered_map>
#include <string>

// This map should be kept in sync with the audio files on the peripheral device.
static std::unordered_map<std::string, int> wav_map = {
    {"first", 0},       // "With assistance, please navigate to" -> aisle (19) -> number (8-17)
    {"final", 1},       // "Obtained final item, please navigate to checkout with assistance"

    {"turn", 2},        // "Please turn" -> left/right/around (3-5)
    {"left", 3},        // "Left"
    {"right", 4},       // "Right"
    {"around", 5},      // "Around"

    {"straight", 6},    // "Please go straight" -> # (8-17) -> meters (7)
    {"meters", 7},      // "Meters"
    {"1",  8},          // "1"
    {"2",  9},         // .
    {"3",  10},         // .
    {"4",  11},         // .
    {"5",  12},         // .
    {"6",  13},         // .
    {"7",  14},         // .
    {"8",  15},         // .
    {"9",  16},         // .
    {"10", 17},         // "10"

    {"item", 18},       // "Item is on" -> shelf (3) -> # (8-17) 
                        // "Item is on" -> left/right (3/4)
    {"aisle", 19},      // "Aisle"
    {"shelf", 20},      // "Shelf"
};
