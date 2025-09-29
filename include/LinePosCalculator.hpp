#pragma once

#include <cmath>

#include <SensorData.hpp>
#include <cfg_sensor.hpp>
#include <utility>

#include <micro/container/set.hpp>
#include <micro/container/vector.hpp>
#include <micro/math/numeric.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/units.hpp>

struct WeightCalculator {
    int8_t radius    = 0;
    float edgeWeight = 0.0f;
    float sumWeight  = 0.0f;

    constexpr WeightCalculator(const float radius)
        : radius(static_cast<int8_t>(micro::ceil(radius))),
          edgeWeight(radius - micro::floor(radius - 0.001f)),
          sumWeight(1.0f + 2 * (this->radius - 1 + this->edgeWeight)) {}

    constexpr WeightCalculator(const float radius, const uint8_t centerIdx)
        : WeightCalculator(radius <= centerIdx ? radius : static_cast<float>(centerIdx)) {}

    constexpr float weight(const int8_t subIdx) const {
        return micro::abs(subIdx) == this->radius ? this->edgeWeight : 1.0f;
    }
};

struct LinePosition {
    micro::millimeter_t pos;
    float probability{};

    bool operator<(const LinePosition& other) const { return this->pos < other.pos; }
    bool operator>(const LinePosition& other) const { return this->pos > other.pos; }
};

using LinePositions = micro::set<LinePosition, micro::Line::MAX_NUM_LINES>;

class LinePosCalculator {
  public:
    explicit LinePosCalculator(const bool whiteLevelCalibrationEnabled);

    LinePositions calculate(const Measurements& measurements, const size_t maxLines);

    static micro::millimeter_t optoIdxToLinePos(const float optoIdx);
    static float linePosToOptoPos(const micro::millimeter_t linePos);

  private:
    using Intensities = std::array<float, cfg::NUM_SENSORS>;

    struct GroupIntensity {
        uint8_t centerIdx;
        float intensity;

        bool operator<(const GroupIntensity& other) const {
            return this->intensity < other.intensity;
        }
        bool operator>(const GroupIntensity& other) const {
            return this->intensity > other.intensity;
        }
    };

    using GroupIntensities =
        micro::vector<GroupIntensity,
                      cfg::NUM_SENSORS - 2 * static_cast<size_t>(micro::ceil(
                                                 cfg::LINE_POS_CALC_INTENSITY_GROUP_RADIUS))>;

    LinePositions runCalculation(const Measurements& measurements, const size_t maxLines);

    void runCalibration(const Measurements& measurements, const size_t maxLines);

    void updateInvalidWhiteLevels(const LinePositions& linePositions);

    Intensities normalize(const Measurements& measurements);

    static GroupIntensities calculateGroupIntensities(const Intensities& intensities);
    static micro::millimeter_t calculateLinePos(const Intensities& intensities,
                                                const uint8_t centerIdx);

    bool whiteLevelCalibrationEnabled_;
    Measurements whiteLevels_;
    micro::vector<Measurements, 200> whiteLevelCalibrationBuffer_;
};
