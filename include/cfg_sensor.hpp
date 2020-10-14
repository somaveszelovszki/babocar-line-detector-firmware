#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr uint8_t MAX_NUM_FILTERED_LINES             = 6;
constexpr uint8_t NUM_SENSORS                        = 48;
constexpr uint8_t LINE_POS_CALC_OFFSET_FILTER_RADIUS = 8;
constexpr micro::millimeter_t MAX_LINE_JUMP          = micro::millimeter_t(8);
constexpr micro::millimeter_t MIN_LINE_DIST          = micro::millimeter_t(25);
constexpr int8_t LINE_FILTER_HYSTERESIS              = 10;
constexpr uint8_t LINE_FILTER_VELO_PEEK_BACK         = 5;
constexpr float MIN_LINE_PROBABILITY                 = 0.4f;
constexpr float MIN_LINE_APPEAR_PROBABILITY          = 0.6f;
constexpr micro::millimeter_t OPTO_ARRAY_LENGTH      = micro::millimeter_t(274.574f);

} // namespace cfg
