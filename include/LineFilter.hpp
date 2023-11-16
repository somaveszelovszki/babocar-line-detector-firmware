#pragma once

#include <etl/circular_buffer.h>

#include <micro/container/set.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/Line.hpp>

#include <cfg_sensor.hpp>
#include <LinePosCalculator.hpp>

#define TRACKED_LINE_ID_INVALID 0
#define TRACKED_LINE_ID_MAX     7

class LineFilter {
public:
    micro::Lines update(const LinePositions& detectedLines);

private:
    struct FilteredLine {
        uint8_t id = 0;
        etl::circular_buffer<micro::millimeter_t, 100> samples;
        micro::millimeter_t estimated;
        int8_t cntr = 0;
        bool isValidated = false;

        micro::millimeter_t current() const;

        micro::millimeter_t current_raw() const { return samples.empty() ? micro::millimeter_t(0) : samples.back(); }

        bool operator<(const FilteredLine& other) const { return current_raw() < other.current_raw(); }
        bool operator>(const FilteredLine& other) const { return current_raw() > other.current_raw(); }

        void increaseCntr() {
            cntr = micro::max<int8_t>(cntr, 0);
            cntr = micro::min<int8_t>(cntr + 1, cfg::LINE_FILTER_HYSTERESIS);
        }

        void decreaseCntr() {
            cntr = micro::min<int8_t>(cntr, 0);
            cntr = micro::max<int8_t>(cntr - 1, -cfg::LINE_FILTER_HYSTERESIS);
        }
    };

    using FilteredLines = micro::set<FilteredLine, cfg::MAX_NUM_FILTERED_LINES>;

    uint8_t generateNewLineId();

    FilteredLines lines_;
};
