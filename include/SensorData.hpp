#pragma once

#include <array>
#include <cfg_sensor.hpp>

#include <micro/utils/types.hpp>

typedef std::array<uint8_t, cfg::NUM_SENSORS> Measurements;
typedef std::array<bool, cfg::NUM_SENSORS> Leds;
