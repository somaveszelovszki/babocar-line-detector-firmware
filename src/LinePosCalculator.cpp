#include <micro/utils/algorithm.hpp>
#include <micro/math/unit_utils.hpp>

#include <cfg_sensor.hpp>
#include <LinePosCalculator.hpp>

using namespace micro;

constexpr uint8_t LinePosCalculator::INTENSITY_GROUP_RADIUS;
constexpr uint8_t LinePosCalculator::POS_CALC_GROUP_RADIUS;
constexpr uint8_t LinePosCalculator::NUM_GROUP_INTENSITIES;

linePositions_t LinePosCalculator::calculate(const measurements_t& measurements) {
    linePositions_t positions;
    const uint16_t average = micro::accumulate(measurements, &measurements[cfg::NUM_SENSORS], static_cast<uint16_t>(0)) / cfg::NUM_SENSORS;

    if (average < 150) {
        uint8_t intensities[cfg::NUM_SENSORS];
        removeOffset(measurements, intensities);
        groupIntensities_t groupIntensities;
        calculateGroupIntensities(intensities, groupIntensities);

        while (positions.size() < positions.capacity() && !groupIntensities.empty()) {
            const groupIntensities_t::const_iterator maxGroupIntensity = groupIntensities.back();
            const millimeter_t linePos = calculateLinePos(intensities, maxGroupIntensity->centerIdx);

            if (std::find_if(positions.begin(), positions.end(), [linePos] (const linePosition_t& pos) {
                return abs(pos.pos - linePos) <= cfg::MIN_LINE_DIST;
            }) == positions.end()) {
                static constexpr uint16_t maxProbabilityGroupIntensity = (2 * INTENSITY_GROUP_RADIUS + 1) * 200;
                const float probability = map(maxGroupIntensity->intensity, groupIntensities.begin()->intensity, maxProbabilityGroupIntensity, 0.0f, 1.0f);
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
        const uint8_t startIdx = i >= cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS ? i - cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS : 0;
        const uint8_t endIdx = i < cfg::NUM_SENSORS - cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS ? i + cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS : cfg::NUM_SENSORS - 1;

        const uint16_t moving_min = *std::min_element(&measurements[startIdx], &measurements[endIdx]);
        result[i] = map<uint16_t, uint8_t>(measurements[i], moving_min, 255, 0, 255);
    }
}

void LinePosCalculator::calculateGroupIntensities(const uint8_t * const intensities, groupIntensities_t& OUT groupIntensities) {
    groupIntensities.resize(NUM_GROUP_INTENSITIES);
    for (uint8_t groupIdx = INTENSITY_GROUP_RADIUS; groupIdx < NUM_GROUP_INTENSITIES; ++groupIdx) {
        uint16_t groupIntensity = 0;
        for (int8_t subIdx = -static_cast<int8_t>(INTENSITY_GROUP_RADIUS); subIdx < static_cast<int8_t>(INTENSITY_GROUP_RADIUS); ++subIdx) {
            groupIntensity += intensities[groupIdx + subIdx];
        }
        groupIntensities[groupIdx] = { groupIdx, groupIntensity };
    }
    groupIntensities.sort();
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
