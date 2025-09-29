#include <LinePosCalculator.hpp>
#include <numeric>

#include <micro/math/unit_utils.hpp>
#include <micro/utils/algorithm.hpp>

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

micro::millimeter_t LinePosCalculator::optoIdxToLinePos(const float optoIdx) {
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

    const auto intensities = normalize(measurements);

    if (std::accumulate(&intensities[0], &intensities[cfg::NUM_SENSORS], 0.0f) / cfg::NUM_SENSORS >
        0.3f) {
        return {};
    }

    auto groupIntensities = calculateGroupIntensities(intensities);

    const float minGroupIntensity =
        std::min_element(groupIntensities.begin(), groupIntensities.end())->intensity;
    uint8_t lastInsertedIdx = 255;

    LinePositions positions;

    while (positions.size() < maxLines && !groupIntensities.empty()) {
        const auto candidate = std::max_element(groupIntensities.begin(), groupIntensities.end());
        const auto [centerIdx, intensity] = *candidate;

        if (micro::abs(static_cast<int32_t>(lastInsertedIdx) - static_cast<int32_t>(centerIdx)) >=
            4) {
            const auto linePos = calculateLinePos(intensities, centerIdx);
            const float probability =
                micro::lerp(intensity, minGroupIntensity, MAX_GROUP_INTENSITY, 0.0f, 1.0f);

            if (probability < cfg::MIN_LINE_PROBABILITY) {
                break;
            }

            if (std::find_if(positions.begin(), positions.end(), [linePos](const auto& pos) {
                    return micro::abs(pos.pos - linePos) <= cfg::MIN_LINE_DIST;
                }) == positions.end()) {
                positions.insert({linePos, probability});
            }

            lastInsertedIdx = candidate->centerIdx;
        }

        groupIntensities.erase(candidate);
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
                      std::max<uint8_t>(sensorIdx, cfg::WHITE_LEVEL_LINE_GROUP_RADIUS) -
                          cfg::WHITE_LEVEL_LINE_GROUP_RADIUS),
            std::next(this->whiteLevels_.begin(),
                      std::min<uint8_t>(sensorIdx + cfg::WHITE_LEVEL_LINE_GROUP_RADIUS + 1,
                                        cfg::NUM_SENSORS))};

        for (Measurements::iterator it = range.first; it != range.second; ++it) {
            *it = whiteLevelMedian;
        }
    }
}

auto LinePosCalculator::normalize(const Measurements& measurements) -> Intensities {
    Intensities intensities;

    // removes sensor-specific offset
    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        intensities[i] =
            micro::lerp<uint8_t>(measurements[i], this->whiteLevels_[i], 255, 0.0f, 1.0f);
    }

    return intensities;
}

auto LinePosCalculator::calculateGroupIntensities(const Intensities& intensities)
    -> GroupIntensities {
    constexpr WeightCalculator CALC(cfg::LINE_POS_CALC_INTENSITY_GROUP_RADIUS);

    GroupIntensities groupIntensities;
    for (uint8_t groupIdx = CALC.radius; groupIdx < cfg::NUM_SENSORS - CALC.radius; ++groupIdx) {
        float groupIntensity = 0.0f;
        for (int8_t subIdx = -CALC.radius; subIdx <= CALC.radius; ++subIdx) {
            groupIntensity += CALC.weight(subIdx) * intensities[groupIdx + subIdx];
        }

        groupIntensities.push_back({groupIdx, groupIntensity / CALC.sumWeight});
    }
    return groupIntensities;
}

micro::millimeter_t LinePosCalculator::calculateLinePos(const Intensities& intensities,
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
