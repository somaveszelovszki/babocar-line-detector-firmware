#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr uint8_t MAX_NUM_FILTERED_LINES             = 6;
constexpr uint8_t NUM_SENSORS                        = 48;
constexpr uint8_t LINE_POS_CALC_OFFSET_FILTER_RADIUS = 3;
constexpr float LINE_POS_CALC_INTENSITY_GROUP_RADIUS = 0.5f;
constexpr float LINE_POS_CALC_GROUP_RADIUS           = 1.0f;
constexpr micro::millimeter_t MAX_LINE_JUMP          = micro::millimeter_t(20);
constexpr micro::millimeter_t MIN_LINE_DIST          = micro::millimeter_t(25);
constexpr int8_t LINE_FILTER_HYSTERESIS              = 7;
constexpr uint8_t LINE_VELO_FILTER_SIZE              = 4;
constexpr uint8_t LINE_POS_FILTER_WINDOW_SIZE        = 1;
constexpr float MIN_LINE_PROBABILITY                 = 0.45f;
constexpr micro::millimeter_t OPTO_ARRAY_LENGTH      = micro::millimeter_t(274.574f);

} // namespace cfg
