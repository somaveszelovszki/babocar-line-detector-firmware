#include <micro/utils/algorithm.hpp>
#include <micro/math/unit_utils.hpp>

#include <cfg_sensor.hpp>
#include <LinePosCalculator.hpp>

using namespace micro;

constexpr uint8_t LinePosCalculator::INTENSITY_GROUP_RADIUS;
constexpr uint8_t LinePosCalculator::POS_CALC_GROUP_RADIUS;
constexpr uint8_t LinePosCalculator::NUM_GROUP_INTENSITIES;

linePositions_t LinePosCalculator::calculate(const measurements_t& measurements) {

    static constexpr uint8_t MAX_VALID_AVERAGE = 150;
    static constexpr uint8_t MAX_PROBABILITY_GROUP_INTENSITY = 200;

    linePositions_t positions;
    const uint8_t average = static_cast<uint8_t>(micro::accumulate(measurements, &measurements[cfg::NUM_SENSORS], static_cast<uint16_t>(0)) / cfg::NUM_SENSORS);

    if (average < MAX_VALID_AVERAGE) {
        uint8_t intensities[cfg::NUM_SENSORS];
        removeOffset(measurements, intensities);
        groupIntensities_t groupIntensities = calculateGroupIntensities(intensities);
        const groupIntensities_t::const_iterator minGroupIntensity = std::min_element(groupIntensities.begin(), groupIntensities.end());

        while (positions.size() < positions.capacity() && !groupIntensities.empty()) {
            const groupIntensities_t::const_iterator maxGroupIntensity = std::max_element(groupIntensities.begin(), groupIntensities.end());
            const millimeter_t linePos = calculateLinePos(intensities, maxGroupIntensity->centerIdx);
            const float probability = map(maxGroupIntensity->intensity, minGroupIntensity->intensity, MAX_PROBABILITY_GROUP_INTENSITY, 0.0f, 1.0f);

            if (probability < cfg::MIN_LINE_PROBABILITY) {
                break;
            }

            if (std::find_if(positions.begin(), positions.end(), [linePos] (const linePosition_t& pos) {
                return abs(pos.pos - linePos) <= cfg::MIN_LINE_DIST;
            }) == positions.end()) {
                positions.insert({ linePos, probability });
            }

            groupIntensities.erase(maxGroupIntensity);
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

void LinePosCalculator::removeOffset(const uint8_t * const measurements, uint8_t * const OUT result) {
    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        const uint8_t startIdx = max<uint8_t>(i, cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS) - cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS;
        const uint8_t endIdx = min<uint8_t>(i + cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS + 1, cfg::NUM_SENSORS);

        const uint16_t moving_min = *std::min_element(&measurements[startIdx], &measurements[endIdx]);
        result[i] = map<uint16_t, uint8_t>(measurements[i], moving_min, 255, 0, 255);
    }
}

LinePosCalculator::groupIntensities_t LinePosCalculator::calculateGroupIntensities(const uint8_t * const intensities) {
    groupIntensities_t groupIntensities;
    for (uint8_t groupIdx = INTENSITY_GROUP_RADIUS; groupIdx < NUM_GROUP_INTENSITIES; ++groupIdx) {
        uint16_t groupIntensity = 0;
        for (int8_t subIdx = -static_cast<int8_t>(INTENSITY_GROUP_RADIUS); subIdx < static_cast<int8_t>(INTENSITY_GROUP_RADIUS); ++subIdx) {
            groupIntensity += intensities[groupIdx + subIdx];
        }
        groupIntensities.push_back({ groupIdx, static_cast<uint8_t>(groupIntensity / (2 * INTENSITY_GROUP_RADIUS - 1)) });
    }
    return groupIntensities;
}

millimeter_t LinePosCalculator::calculateLinePos(const uint8_t * const intensities, const uint8_t centerIdx) {

    const uint8_t startIdx = max(centerIdx, POS_CALC_GROUP_RADIUS) - POS_CALC_GROUP_RADIUS;
    const uint8_t lastIdx  = min(centerIdx + POS_CALC_GROUP_RADIUS, cfg::NUM_SENSORS - 1);

    uint16_t sum = 0;
    uint16_t sumW = 0;

    for (uint8_t i = startIdx; i <= lastIdx; ++i) {
        const uint8_t m = intensities[i];
        sum += m;
        sumW += m * i;
    }

    return optoIdxToLinePos(static_cast<float>(sumW) / sum);
}
