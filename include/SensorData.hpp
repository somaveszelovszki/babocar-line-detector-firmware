#pragma once

#include <micro/utils/types.hpp>

#include <cfg_sensor.hpp>

#include <array>

typedef std::array<uint8_t, cfg::NUM_SENSORS> Measurements;
typedef std::array<bool, cfg::NUM_SENSORS> Leds;

struct SensorControlData {
    Leds leds;
    bool scanEnabled        = false;
    uint8_t scanRangeCenter = cfg::NUM_SENSORS / 2;
    uint8_t scanRangeRadius = 0;
};
