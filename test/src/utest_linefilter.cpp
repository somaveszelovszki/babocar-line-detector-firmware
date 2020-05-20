#include <micro/math/numeric.hpp>
#include <micro/test/utils.hpp>
#include <LineFilter.hpp>

#define PRINT_MEAS false
#include <cmath>

#if PRINT_MEAS
#include <iomanip>
#include <iostream>
#include <string>
#endif // PRINT_MEAS

using namespace micro;

namespace {

constexpr uint32_t NUM_TESTS_PER_SCENARIO = 1;

linePositions_t addNoise(const linePositions_t& linePositions) {

    static constexpr millimeter_t MAX_RAND_NOISE = { 3 };

    linePositions_t result = linePositions;

    for (linePosition_t& linePos : result) {
        const millimeter_t noise = map<uint32_t, millimeter_t>(rand() % 10000, 0, 10000, -MAX_RAND_NOISE, MAX_RAND_NOISE);
        linePos.pos += noise;
    }

    return result;
}

linePositions_t move(const linePositions_t& linePositions, const millimeter_t distance) {

    linePositions_t result = linePositions;

    for (linePosition_t& linePos : result) {
        linePos.pos += distance;
    }

    return result;
}

} // namespace

TEST(LineFilter, one_line_few_detections) {
    linePositions_t linePositions = { { millimeter_t(0), 1.0f } };

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS - 1; ++i) {
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(0, lines.size());
}

TEST(LineFilter, one_line) {
    linePositions_t linePositions = { { millimeter_t(0), 1.0f } };

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ_UNIT(linePositions[i].pos, lines[i].pos);
        EXPECT_EQ(i + 1, lines[i].id);
    }
}

TEST(LineFilter, one_line_noise) {
    linePositions_t linePositions_base = { { millimeter_t(0), 1.0f } };
    linePositions_t linePositions;

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        linePositions = addNoise(linePositions_base);
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ_UNIT(linePositions[i].pos, lines[i].pos);
        EXPECT_EQ(i + 1, lines[i].id);
    }
}

TEST(LineFilter, one_line_noise_false_positives) {
    linePositions_t linePositions_base = { { millimeter_t(0), 1.0f } };
    linePositions_t linePositionsFalsePositives_base = { { millimeter_t(0), 1.0f }, { millimeter_t(50), 1.0f } };
    linePositions_t linePositions;

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        linePositions = addNoise(i == 0 ? linePositionsFalsePositives_base : linePositions_base);
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ_UNIT(linePositions[i].pos, lines[i].pos);
        EXPECT_EQ(i + 1, lines[i].id);
    }
}

TEST(LineFilter, one_line_true_negatives) {
    linePositions_t linePositions = { { millimeter_t(0), 1.0f } };
    linePositions_t linePositionsTrueNegatives = {};

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ_UNIT(linePositions[i].pos, lines[i].pos);
        EXPECT_EQ(i + 1, lines[i].id);
    }

    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS - 1; ++i) {
        lines = lineFilter.update(linePositionsTrueNegatives);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ_UNIT(linePositions[i].pos, lines[i].pos);
        EXPECT_EQ(i + 1, lines[i].id);
    }

    lines = lineFilter.update(linePositionsTrueNegatives);

    EXPECT_EQ(0, lines.size());
}

TEST(LineFilter, one_moving_line_true_negatives) {

    static constexpr millimeter_t MOVE_DISTANCE = { 5 };

    linePositions_t linePositions = { { millimeter_t(0), 1.0f } };
    linePositions_t linePositionsTrueNegatives = {};

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        linePositions = move(linePositions, MOVE_DISTANCE);
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ_UNIT(linePositions[i].pos, lines[i].pos);
        EXPECT_EQ(i + 1, lines[i].id);
    }

    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS - 1; ++i) {
        linePositions = move(linePositions, MOVE_DISTANCE);
        lines = lineFilter.update(linePositionsTrueNegatives);
    }

    EXPECT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        EXPECT_NEAR_UNIT(linePositions[i].pos, lines[i].pos, millimeter_t(1));
        EXPECT_EQ(i + 1, lines[i].id);
    }

    linePositions = move(linePositions, MOVE_DISTANCE);
    lines = lineFilter.update(linePositionsTrueNegatives);

    EXPECT_EQ(0, lines.size());
}
