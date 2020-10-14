#pragma once

#include <micro/container/bit_array.hpp>
#include <micro/utils/types.hpp>

#include <cfg_sensor.hpp>

typedef uint8_t Measurements[cfg::NUM_SENSORS];
typedef micro::bit_array<cfg::NUM_SENSORS> Leds;

struct SensorControlData {
    Leds leds;
    uint8_t scanRangeCenter = cfg::NUM_SENSORS / 2;
    uint8_t scanRangeRadius = 0;
};
