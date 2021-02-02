#pragma once

#include <micro/container/vec.hpp>
#include <micro/math/numeric.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/units.hpp>

#include <cfg_sensor.hpp>
#include <SensorData.hpp>

#include <utility>

struct WeightCalculator {
    int8_t radius    = 0;
    float lastWeight = 0.0f;
    float sumWeight  = 0.0f;

    constexpr WeightCalculator(const float radius)
        : radius(micro::round_up(radius))
        , lastWeight(radius - micro::round_down(radius - 0.001f))
        , sumWeight(1.0f + 2 * (this->radius - 1 + this->lastWeight)) {}

    constexpr WeightCalculator(const float radius, const uint8_t centerIdx)
        : WeightCalculator(radius <= centerIdx ? radius : static_cast<float>(centerIdx)) {}

    constexpr float weight(const int8_t subIdx) const {
        return micro::abs(subIdx) == this->radius ? this->lastWeight : 1.0f;
    }
};

struct LinePosition {
    micro::millimeter_t pos;
    float probability;

    bool operator<(const LinePosition& other) const { return this->pos < other.pos; }
    bool operator>(const LinePosition& other) const { return this->pos > other.pos; }
};

typedef micro::sorted_vec<LinePosition, micro::Line::MAX_NUM_LINES> LinePositions;

class LinePosCalculator {
public:
    explicit LinePosCalculator(const bool whiteLevelCalibrationEnabled);

    LinePositions calculate(const Measurements& measurements);

    static micro::millimeter_t optoIdxToLinePos(const float optoIdx);
    static float linePosToOptoPos(const micro::millimeter_t linePos);

private:
    struct groupIntensity_t {
        uint8_t centerIdx;
        float intensity;

        bool operator<(const groupIntensity_t& other) const { return this->intensity < other.intensity; }
        bool operator>(const groupIntensity_t& other) const { return this->intensity > other.intensity; }
    };

    typedef micro::vec<groupIntensity_t, cfg::NUM_SENSORS - 2 * micro::round_up(cfg::LINE_POS_CALC_INTENSITY_GROUP_RADIUS)> groupIntensities_t;

    LinePositions runCalculation(const Measurements& measurements);

    void runCalibration(const Measurements& measurements);

    void updateInvalidWhiteLevels(const LinePositions& linePositions);

    void normalize(const Measurements& measurements, float * const OUT result);

    static groupIntensities_t calculateGroupIntensities(const float * const intensities);
    static micro::millimeter_t calculateLinePos(const float * const intensities, const uint8_t centerIdx);

    bool whiteLevelCalibrationEnabled_;
    Measurements whiteLevels_;
    micro::vec<Measurements, 200> whiteLevelCalibrationBuffer_;
};
