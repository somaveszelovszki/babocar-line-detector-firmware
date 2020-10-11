#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/units.hpp>

#include <cfg_sensor.hpp>
#include <SensorData.hpp>

#include <utility>

struct LinePosition {
    micro::millimeter_t pos;
    float probability;

    bool operator<(const LinePosition& other) const { return this->pos < other.pos; }
    bool operator>(const LinePosition& other) const { return this->pos > other.pos; }
};

typedef micro::sorted_vec<LinePosition, micro::Line::MAX_NUM_LINES> LinePositions;

class LinePosCalculator {
public:
    explicit LinePosCalculator(const std::pair<uint8_t, uint8_t> sensorLimits[cfg::NUM_SENSORS]);

    LinePositions calculate(const Measurements& measurements);

    static micro::millimeter_t optoIdxToLinePos(const float optoIdx);
    static float linePosToOptoPos(const micro::millimeter_t linePos);

private:
    static constexpr uint8_t INTENSITY_GROUP_RADIUS = 1;
    static constexpr uint8_t POS_CALC_GROUP_RADIUS  = 2;
    static constexpr uint8_t NUM_GROUP_INTENSITIES  = cfg::NUM_SENSORS - INTENSITY_GROUP_RADIUS * 2;

    struct groupIntensity_t {
        uint8_t centerIdx;
        float intensity;

        bool operator<(const groupIntensity_t& other) const { return this->intensity < other.intensity; }
        bool operator>(const groupIntensity_t& other) const { return this->intensity > other.intensity; }
    };

    typedef micro::vec<groupIntensity_t, NUM_GROUP_INTENSITIES> groupIntensities_t;

    void normalize(const uint8_t * const measurements, float * const OUT result);

    static groupIntensities_t calculateGroupIntensities(const float * const intensities);
    static micro::millimeter_t calculateLinePos(const float * const intensities, const uint8_t centerIdx);

    const std::pair<uint8_t, uint8_t> *sensorLimits_;
};
