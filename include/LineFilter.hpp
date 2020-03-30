#pragma once

#include <micro/container/vec.hpp>
#include <micro/math/unit_utils.hpp>

#include <cfg_sensor.hpp>
#include <LinePosCalculator.hpp>

#define TRACKED_LINE_ID_INVALID 0
#define TRACKED_LINE_ID_MAX     7

struct trackedLine_t {
    micro::millimeter_t pos;
    uint8_t id : 3;

    bool operator<(const trackedLine_t& other) const { return this->pos < other.pos; }
    bool operator>(const trackedLine_t& other) const { return this->pos > other.pos; }
};

class LineFilter {
public:
    micro::sorted_vec<trackedLine_t, cfg::MAX_NUM_LINES> update(const linePositions_t& detectedLines);

private:
    struct filteredLine_t {
        trackedLine_t current;
        micro::millimeter_t prev;
        uint8_t cntr = 0;

        bool operator<(const filteredLine_t& other) const { return this->current.pos < other.current.pos; }
        bool operator>(const filteredLine_t& other) const { return this->current.pos > other.current.pos; }

        micro::millimeter_t distance(const micro::millimeter_t& line) const {
            return micro::abs(this->current.pos - line);
        }

        micro::millimeter_t distance(const trackedLine_t& line) const {
            return this->distance(line.pos);
        }
    };

    typedef micro::sorted_vec<filteredLine_t, cfg::MAX_NUM_FILTERED_LINES> filteredLines_t;

    filteredLines_t lines_;
};
