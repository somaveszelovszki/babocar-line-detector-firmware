#include <micro/utils/algorithm.hpp>
#include <micro/math/unit_utils.hpp>

#include <cfg_sensor.hpp>
#include <LinePosCalculator.hpp>

#include <numeric>

using namespace micro;

constexpr std::pair<uint8_t, uint8_t> LinePosCalculator::INTENSITY_GROUP_RADIUS;
constexpr uint8_t LinePosCalculator::POS_CALC_GROUP_RADIUS;
constexpr uint8_t LinePosCalculator::NUM_GROUP_INTENSITIES;

LinePosCalculator::LinePosCalculator(const std::pair<uint8_t, uint8_t> sensorLimits[cfg::NUM_SENSORS])
    : sensorLimits_(sensorLimits) {}

LinePositions LinePosCalculator::calculate(const Measurements& measurements) {

    static constexpr float MAX_VALID_AVERAGE = 0.5f;

    LinePositions positions;

    float intensities[cfg::NUM_SENSORS];
    this->normalize(measurements, intensities);

    const float average = std::accumulate(intensities, &intensities[cfg::NUM_SENSORS], 0.0f) / cfg::NUM_SENSORS;
    if (average < MAX_VALID_AVERAGE) {
        groupIntensities_t groupIntensities = calculateGroupIntensities(intensities);

        const float minGroupIntensity = std::min_element(groupIntensities.begin(), groupIntensities.end())->intensity;
        uint8_t lastInsertedIdx       = 255;

        while (positions.size() < positions.capacity() && !groupIntensities.empty()) {

            const groupIntensities_t::const_iterator candidate = std::max_element(groupIntensities.begin(), groupIntensities.end());

            if (micro::abs(static_cast<int32_t>(lastInsertedIdx) - static_cast<int32_t>(candidate->centerIdx)) >= 4) {
                const millimeter_t linePos = calculateLinePos(intensities, candidate->centerIdx);
                const float probability = map(candidate->intensity, minGroupIntensity, 1.0f, 0.0f, 1.0f);

                if (probability < cfg::MIN_LINE_PROBABILITY) {
                    break;
                }

                if (std::find_if(positions.begin(), positions.end(), [linePos] (const LinePosition& pos) {
                    return abs(pos.pos - linePos) <= cfg::MIN_LINE_DIST;
                }) == positions.end()) {
                    positions.insert({ linePos, probability });
                }

                lastInsertedIdx = candidate->centerIdx;
            }

            groupIntensities.erase(candidate);
        }
    }

    return positions;
}

millimeter_t LinePosCalculator::optoIdxToLinePos(const float optoIdx) {
    return map(optoIdx, 0.0f, cfg::NUM_SENSORS - 1.0f, -cfg::OPTO_ARRAY_LENGTH / 2, cfg::OPTO_ARRAY_LENGTH / 2);
}

float LinePosCalculator::linePosToOptoPos(const micro::millimeter_t linePos) {
    return map(linePos, -cfg::OPTO_ARRAY_LENGTH / 2, cfg::OPTO_ARRAY_LENGTH / 2, 0.0f, cfg::NUM_SENSORS - 1.0f);
}

void LinePosCalculator::normalize(const uint8_t * const measurements, float * const OUT result) {

    float scaled[cfg::NUM_SENSORS];

    // removes sensor-specific offset
    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        scaled[i] = micro::map(measurements[i], this->sensorLimits_[i].first, this->sensorLimits_[i].second, 0.0f, 1.0f);
    }

    // removes dynamic light-related offset, that applies to the neighboring sensors
    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        const uint8_t startIdx = max<uint8_t>(i, cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS) - cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS;
        const uint8_t endIdx = min<uint8_t>(i + cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS + 1, cfg::NUM_SENSORS);

        const float moving_min = *std::min_element(&scaled[startIdx], &scaled[endIdx]);
        result[i] = map(scaled[i], moving_min, 1.0f, 0.0f, 1.0f);
    }
}

LinePosCalculator::groupIntensities_t LinePosCalculator::calculateGroupIntensities(const float * const intensities) {
    groupIntensities_t groupIntensities;
    for (uint8_t groupIdx = INTENSITY_GROUP_RADIUS.first; groupIdx < cfg::NUM_SENSORS - INTENSITY_GROUP_RADIUS.second; ++groupIdx) {
        float groupIntensity = 0.0f;
        for (int8_t subIdx = -static_cast<int8_t>(INTENSITY_GROUP_RADIUS.first); subIdx <= static_cast<int8_t>(INTENSITY_GROUP_RADIUS.second); ++subIdx) {
            groupIntensity += intensities[groupIdx + subIdx];
        }

        groupIntensities.push_back({ groupIdx, groupIntensity / (INTENSITY_GROUP_RADIUS.first + 1 + INTENSITY_GROUP_RADIUS.second) });
    }
    return groupIntensities;
}

millimeter_t LinePosCalculator::calculateLinePos(const float * const intensities, const uint8_t centerIdx) {

    const uint8_t startIdx = max(centerIdx, POS_CALC_GROUP_RADIUS) - POS_CALC_GROUP_RADIUS;
    const uint8_t lastIdx  = min(centerIdx + POS_CALC_GROUP_RADIUS, cfg::NUM_SENSORS - 1);

    float sum = 0;
    float sumW = 0;

    for (uint8_t i = startIdx; i <= lastIdx; ++i) {
        const float m = intensities[i];
        sum += m;
        sumW += m * i;
    }

    return optoIdxToLinePos(sumW / sum);
}
