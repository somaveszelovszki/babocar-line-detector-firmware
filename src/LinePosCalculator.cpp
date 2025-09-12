#include <LinePosCalculator.hpp>
#include <numeric>

#include <micro/math/unit_utils.hpp>
#include <micro/utils/algorithm.hpp>

using namespace micro;

LinePosCalculator::LinePosCalculator(const bool whiteLevelCalibrationEnabled)
    : whiteLevelCalibrationEnabled_(whiteLevelCalibrationEnabled) {
    this->whiteLevels_.fill(0);
}

LinePositions LinePosCalculator::calculate(const Measurements& measurements,
                                           const size_t maxLines) {
    LinePositions positions;

    if (!this->whiteLevelCalibrationEnabled_ || this->whiteLevelCalibrationBuffer_.size() ==
                                                    this->whiteLevelCalibrationBuffer_.capacity()) {
        positions = this->runCalculation(measurements, maxLines);
    } else {
        this->runCalibration(measurements, maxLines);
    }

    return positions;
}

millimeter_t LinePosCalculator::optoIdxToLinePos(const float optoIdx) {
    return micro::lerp(optoIdx, 0.0f, cfg::NUM_SENSORS - 1.0f, -cfg::OPTO_ARRAY_LENGTH / 2,
                       cfg::OPTO_ARRAY_LENGTH / 2);
}

float LinePosCalculator::linePosToOptoPos(const micro::millimeter_t linePos) {
    return micro::lerp(linePos, -cfg::OPTO_ARRAY_LENGTH / 2, cfg::OPTO_ARRAY_LENGTH / 2, 0.0f,
                       cfg::NUM_SENSORS - 1.0f);
}

LinePositions LinePosCalculator::runCalculation(const Measurements& measurements,
                                                const size_t maxLines) {
    static constexpr float MAX_GROUP_INTENSITY =
        1.0f / (1.0f + cfg::LINE_POS_CALC_INTENSITY_GROUP_RADIUS);

    LinePositions positions;

    float intensities[cfg::NUM_SENSORS];
    this->normalize(measurements, intensities);

    if (std::accumulate(&intensities[0], &intensities[cfg::NUM_SENSORS], 0.0f) / cfg::NUM_SENSORS <
        0.3f) {
        auto groupIntensities = calculateGroupIntensities(intensities);

        const float minGroupIntensity =
            std::min_element(groupIntensities.begin(), groupIntensities.end())->intensity;
        uint8_t lastInsertedIdx = 255;

        while (positions.size() < maxLines && !groupIntensities.empty()) {
            const auto candidate =
                std::max_element(groupIntensities.begin(), groupIntensities.end());

            if (micro::abs(static_cast<int32_t>(lastInsertedIdx) -
                           static_cast<int32_t>(candidate->centerIdx)) >= 4) {
                const millimeter_t linePos = calculateLinePos(intensities, candidate->centerIdx);
                const float probability    = micro::lerp(candidate->intensity, minGroupIntensity,
                                                         MAX_GROUP_INTENSITY, 0.0f, 1.0f);

                if (probability < cfg::MIN_LINE_PROBABILITY) {
                    break;
                }

                if (std::find_if(positions.begin(), positions.end(), [linePos](const auto& pos) {
                        return abs(pos.pos - linePos) <= cfg::MIN_LINE_DIST;
                    }) == positions.end()) {
                    positions.insert({linePos, probability});
                }

                lastInsertedIdx = candidate->centerIdx;
            }

            groupIntensities.erase(candidate);
        }
    }

    return positions;
}

void LinePosCalculator::runCalibration(const Measurements& measurements, const size_t maxLines) {
    this->whiteLevelCalibrationBuffer_.push_back(measurements);
    if (this->whiteLevelCalibrationBuffer_.size() ==
        this->whiteLevelCalibrationBuffer_.capacity()) {
        const LinePositions linePositions = this->runCalculation(measurements, maxLines);

        for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
            float sum = 0.0f;
            for (const Measurements& meas : this->whiteLevelCalibrationBuffer_) {
                sum += meas[i];
            }
            this->whiteLevels_[i] = std::lround(sum / this->whiteLevelCalibrationBuffer_.size());
        }

        this->updateInvalidWhiteLevels(linePositions);
    }
}

void LinePosCalculator::updateInvalidWhiteLevels(const LinePositions& linePositions) {
    Measurements sortedWhiteLevels;
    std::copy(this->whiteLevels_.begin(), this->whiteLevels_.end(), sortedWhiteLevels.begin());
    std::sort(sortedWhiteLevels.begin(), sortedWhiteLevels.end());
    const uint8_t whiteLevelMedian = sortedWhiteLevels[cfg::NUM_SENSORS / 2];

    for (const LinePosition& linePos : linePositions) {
        const uint8_t sensorIdx = std::lround(linePosToOptoPos(linePos.pos));

        const std::pair<Measurements::iterator, Measurements::iterator> range = {
            std::next(this->whiteLevels_.begin(),
                      max<uint8_t>(sensorIdx, cfg::WHITE_LEVEL_LINE_GROUP_RADIUS) -
                          cfg::WHITE_LEVEL_LINE_GROUP_RADIUS),
            std::next(this->whiteLevels_.begin(),
                      min<uint8_t>(sensorIdx + cfg::WHITE_LEVEL_LINE_GROUP_RADIUS + 1,
                                   cfg::NUM_SENSORS))};

        for (Measurements::iterator it = range.first; it != range.second; ++it) {
            *it = whiteLevelMedian;
        }
    }
}

void LinePosCalculator::normalize(const Measurements& measurements, float* const OUT result) {
    float scaled[cfg::NUM_SENSORS];

    // removes sensor-specific offset
    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        scaled[i] = micro::lerp<uint8_t>(measurements[i], this->whiteLevels_[i], 255, 0.0f, 1.0f);
    }

    // removes dynamic light-related offset, that applies to the adjacent sensors
    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        const uint8_t startIdx = max<uint8_t>(i, cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS) -
                                 cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS;
        const uint8_t endIdx =
            min<uint8_t>(i + cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS + 1, cfg::NUM_SENSORS);

        std::array<float, 2 * cfg::LINE_POS_CALC_OFFSET_FILTER_RADIUS + 1> group;
        std::copy(&scaled[startIdx], &scaled[endIdx], group.begin());
        std::sort(group.begin(), std::next(group.begin(), endIdx - startIdx));

        result[i] = micro::lerp(scaled[i], group[group.size() / 3], 1.0f, 0.0f, 1.0f);
    }
}

LinePosCalculator::groupIntensities_t
LinePosCalculator::calculateGroupIntensities(const float* const intensities) {
    static constexpr WeightCalculator CALC(cfg::LINE_POS_CALC_INTENSITY_GROUP_RADIUS);

    groupIntensities_t groupIntensities;
    for (uint8_t groupIdx = CALC.radius; groupIdx < cfg::NUM_SENSORS - CALC.radius; ++groupIdx) {
        float groupIntensity = 0.0f;
        for (int8_t subIdx = -CALC.radius; subIdx <= CALC.radius; ++subIdx) {
            groupIntensity += CALC.weight(subIdx) * intensities[groupIdx + subIdx];
        }

        groupIntensities.push_back({groupIdx, groupIntensity / CALC.sumWeight});
    }
    return groupIntensities;
}

millimeter_t LinePosCalculator::calculateLinePos(const float* const intensities,
                                                 const uint8_t centerIdx) {
    const WeightCalculator calc(cfg::LINE_POS_CALC_GROUP_RADIUS, centerIdx);

    float sum  = 0;
    float sumW = 0;

    for (int8_t subIdx = -calc.radius; subIdx <= calc.radius; ++subIdx) {
        const uint8_t idx = centerIdx + subIdx;
        const float m     = intensities[idx];
        const float w     = calc.weight(subIdx);

        sum += m * w;
        sumW += m * w * idx;
    }

    return optoIdxToLinePos(sumW / sum);
}
