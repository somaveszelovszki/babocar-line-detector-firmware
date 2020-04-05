#pragma once

#include <micro/utils/units.hpp>

namespace globals {

extern uint8_t panelId;
extern bool isConnected;
extern bool indicatorLedsEnabled;
extern uint8_t scanRangeCenter;
extern uint8_t scanRangeRadius; // Radius of the line scan (number of sensors to each direction) - 0 means all sensors

} // namespace globals
