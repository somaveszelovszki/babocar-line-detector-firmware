#include <globals.hpp>

namespace globals {

extern uint8_t panelId    = 0; // TODO join with panel_checkversion
bool isConnected          = false;
bool indicatorLedsEnabled = true;
uint8_t scanRangeCenter   = 0;
uint8_t scanRangeRadius   = 0;

}  // namespace globals
