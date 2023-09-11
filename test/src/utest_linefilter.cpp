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

LinePositions addNoise(const LinePositions& linePositions) {

    static constexpr millimeter_t MAX_RAND_NOISE = { 3 };

    LinePositions result = linePositions;

    for (LinePosition& linePos : result) {
        const millimeter_t noise = map<uint32_t, millimeter_t>(rand() % 10000, 0, 10000, -MAX_RAND_NOISE, MAX_RAND_NOISE);
        linePos.pos += noise;
    }

    return result;
}

LinePositions move(const LinePositions& linePositions, const millimeter_t distance) {

    LinePositions result = linePositions;

    for (LinePosition& linePos : result) {
        linePos.pos += distance;
    }

    return result;
}

void expectEq(const LinePositions& linePositions, const Lines& lines) {
    ASSERT_EQ(linePositions.size(), lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        const auto& linePos = *std::next(linePositions.begin(), i);
        const auto& line = *std::next(lines.begin(), i);
        EXPECT_NEAR_UNIT(linePos.pos, line.pos, millimeter_t(10));
        EXPECT_EQ(i + 1, line.id);
    }
}

} // namespace

TEST(LineFilter, one_line_few_detections) {
    LinePositions linePositions = { { millimeter_t(0), 1.0f } };

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS - 1; ++i) {
        lines = lineFilter.update(linePositions);
    }

    EXPECT_EQ(0, lines.size());
}

TEST(LineFilter, one_line) {
    LinePositions linePositions = { { millimeter_t(0), 1.0f } };

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        lines = lineFilter.update(linePositions);
    }

    expectEq(linePositions, lines);
}

TEST(LineFilter, one_line_noise) {
    LinePositions linePositions_base = { { millimeter_t(0), 1.0f } };
    LinePositions linePositions;

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        linePositions = addNoise(linePositions_base);
        lines = lineFilter.update(linePositions);
    }

    expectEq(linePositions, lines);
}

TEST(LineFilter, one_line_noise_false_positives) {
    LinePositions linePositions_base = { { millimeter_t(0), 1.0f } };
    LinePositions linePositionsFalsePositives_base = { { millimeter_t(0), 1.0f }, { millimeter_t(50), 1.0f } };
    LinePositions linePositions;

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        linePositions = addNoise(i == 0 ? linePositionsFalsePositives_base : linePositions_base);
        lines = lineFilter.update(linePositions);
    }

    expectEq(linePositions, lines);
}

TEST(LineFilter, one_line_true_negatives) {
    LinePositions linePositions = { { millimeter_t(0), 1.0f } };
    LinePositions linePositionsTrueNegatives = {};

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        lines = lineFilter.update(linePositions);
    }

    expectEq(linePositions, lines);

    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS - 1; ++i) {
        lines = lineFilter.update(linePositionsTrueNegatives);
    }

    expectEq(linePositions, lines);

    lines = lineFilter.update(linePositionsTrueNegatives);

    EXPECT_EQ(0, lines.size());
}

TEST(LineFilter, one_moving_line_true_negatives) {

    static constexpr millimeter_t MOVE_DISTANCE = { 1 };

    LinePositions linePositions = { { millimeter_t(0), 1.0f } };
    LinePositions linePositionsTrueNegatives = {};

    LineFilter lineFilter;
    Lines lines;
    
    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS; ++i) {
        linePositions = move(linePositions, MOVE_DISTANCE);
        lines = lineFilter.update(linePositions);
    }

    expectEq(linePositions, lines);

    for (uint32_t i = 0; i < cfg::LINE_FILTER_HYSTERESIS - 1; ++i) {
        linePositions = move(linePositions, MOVE_DISTANCE);
        lines = lineFilter.update(linePositionsTrueNegatives);
    }

    expectEq(linePositions, lines);

    linePositions = move(linePositions, MOVE_DISTANCE);
    lines = lineFilter.update(linePositionsTrueNegatives);

    EXPECT_EQ(0, lines.size());
}
