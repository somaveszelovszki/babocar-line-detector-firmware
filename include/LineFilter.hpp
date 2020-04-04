#pragma once

#include <micro/container/infinite_buffer.hpp>
#include <micro/container/vec.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/Line.hpp>

#include <cfg_sensor.hpp>
#include <LinePosCalculator.hpp>

#define TRACKED_LINE_ID_INVALID 0
#define TRACKED_LINE_ID_MAX     7

class LineFilter {
public:
    micro::Lines update(const linePositions_t& detectedLines);

private:
    typedef micro::infinite_buffer<micro::millimeter_t, 10> linePosSamples_t;

    struct filteredLine_t {
        uint8_t id : 3;
        linePosSamples_t samples;
        micro::millimeter_t estimated;
        int8_t cntr = 0;
        bool isValidated;

        const micro::millimeter_t& current() const { return this->samples.peek_back(0); }
        micro::millimeter_t& current() { return this->samples.peek_back(0); }

        bool operator<(const filteredLine_t& other) const { return this->current() < other.current(); }
        bool operator>(const filteredLine_t& other) const { return this->current() > other.current(); }

        void increaseCntr() {
            cntr = micro::max<int8_t>(cntr, 0);
            cntr = micro::min<int8_t>(cntr + 1, cfg::LINE_FILTER_HYSTERESIS);
        }

        void decreaseCntr() {
            cntr = micro::min<int8_t>(cntr, 0);
            cntr = micro::max<int8_t>(cntr - 1, -cfg::LINE_FILTER_HYSTERESIS);
        }
    };

    typedef micro::sorted_vec<filteredLine_t, cfg::MAX_NUM_FILTERED_LINES> filteredLines_t;

    uint8_t generateNewLineId();

    filteredLines_t lines_;
};
