#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/units.hpp>
#include <cfg_sensor.hpp>
#include <SensorHandler.hpp>

struct linePosition_t {
    micro::millimeter_t pos;
    float probability;

    bool operator<(const linePosition_t& other) const { return this->pos < other.pos; }
    bool operator>(const linePosition_t& other) const { return this->pos > other.pos; }
};

typedef micro::sorted_vec<linePosition_t, cfg::MAX_NUM_LINES> linePositions_t;

class LinePosCalculator {
public:
    linePositions_t calculate(const measurements_t& measurements);

    static micro::millimeter_t optoIdxToLinePos(const float optoIdx);
    static float linePosToOptoPos(const micro::millimeter_t linePos);

private:
    static constexpr uint8_t INTENSITY_GROUP_RADIUS = 1;
    static constexpr uint8_t POS_CALC_GROUP_RADIUS  = 2;
    static constexpr uint8_t NUM_GROUP_INTENSITIES  = cfg::NUM_SENSORS - INTENSITY_GROUP_RADIUS * 2;

    struct groupIntensity_t {
        uint8_t centerIdx;
        uint8_t intensity;

        bool operator<(const groupIntensity_t& other) const { return this->intensity < other.intensity; }
        bool operator>(const groupIntensity_t& other) const { return this->intensity > other.intensity; }
    };

    typedef micro::vec<groupIntensity_t, NUM_GROUP_INTENSITIES> groupIntensities_t;

    static void removeOffset(const uint8_t * const measurements, uint8_t * const OUT result);
    static groupIntensities_t calculateGroupIntensities(const uint8_t * const intensities);
    static micro::millimeter_t calculateLinePos(const uint8_t * const intensities, const uint8_t centerIdx);
};
