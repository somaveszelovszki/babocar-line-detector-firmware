#pragma once

#include <micro/container/bit_array.hpp>
#include <micro/utils/types.hpp>

#include <cfg_sensor.hpp>

typedef uint8_t measurements_t[cfg::NUM_SENSORS];
typedef micro::bit_array<cfg::NUM_SENSORS> leds_t;

struct SensorHandlerData {
    leds_t leds;
    uint8_t scanRangeCenter;
};
