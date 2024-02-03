#include <micro/container/vector.hpp>
#include <micro/math/numeric.hpp>
#include <micro/test/utils.hpp>

#include <LinePosCalculator.hpp>

#define PRINT_MEASUREMENTS false
#include <cmath>

#if PRINT_MEASUREMENTS
#include <iomanip>
#include <iostream>
#include <string>
#endif // PRINT_MEASUREMENTS

using namespace micro;

namespace {

constexpr uint32_t NUM_TESTS_PER_SCENARIO = 10000;

void createMeasurements(const micro::vector<millimeter_t, Line::MAX_NUM_LINES>& lines, Measurements& meas) {
    static constexpr double RANDOM_WEIGHT = 0.25;
    static constexpr double SIGMA = 1.0;
    static constexpr double MAX_Z_SCORE = 1.0 / (SIGMA * std::sqrt(2 * M_PI));

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        meas[i] = 0;
    }

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        for (millimeter_t linePos : lines) {
            const double z_score = (i - LinePosCalculator::linePosToOptoPos(linePos)) / SIGMA;
            const double value = 1.0 / (SIGMA * std::sqrt(2 * M_PI)) * exp(-0.5 * z_score * z_score);
            const float random_mul = micro::lerp<uint32_t, double>(rand() % 10000, 0, 10000, 1 - RANDOM_WEIGHT, 1 + RANDOM_WEIGHT);
            const uint8_t incr = micro::clamp<int32_t>(micro::lerp(value / MAX_Z_SCORE, 0.0, 1.0, 0, 255) * random_mul, 0, 255);
            meas[i] = std::numeric_limits<uint8_t>::max() - incr > meas[i] ? meas[i] + incr : std::numeric_limits<uint8_t>::max();
        }
    }
}

void test(const micro::vector<millimeter_t, Line::MAX_NUM_LINES>& lines) {
    LinePosCalculator linePosCalculator(false);
    Measurements measurements;

    for (uint32_t i = 0; i < NUM_TESTS_PER_SCENARIO; ++i) {
        createMeasurements(lines, measurements);

#if PRINT_MEASUREMENTS
        std::cout << "Test measurements:";
        for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
            std::cout << std::setw(3) << std::to_string(measurements[i]);
            if (i < cfg::NUM_SENSORS - 1) {
                std::cout << ",";
            } else {
                std::cout << std::endl;
            }
        }
#endif // PRINT_MEASUREMENTS

        const auto linePositions = linePosCalculator.calculate(measurements, Line::MAX_NUM_LINES);
        ASSERT_EQ(lines.size(), linePositions.size());
        size_t j = 0;
        for (const auto& line : linePositions) {
            EXPECT_NEAR_UNIT(lines[j], line.pos, millimeter_t(4));
            EXPECT_LE(0.7f, line.probability);
            j++;
        }
    }
}

} // namespace

TEST(LinePosCalculator, one_line_center) {
    test({ millimeter_t(0) });
}

TEST(LinePosCalculator, one_line_left) {
    test({ millimeter_t(-100) });
}

TEST(LinePosCalculator, one_line_right) {
    test({ millimeter_t(100) });
}

TEST(LinePosCalculator, two_lines_close_center) {
    test({ millimeter_t(-10), millimeter_t(28) });
}

TEST(LinePosCalculator, two_lines_close_left) {
    test({ millimeter_t(-120), millimeter_t(-90) });
}

TEST(LinePosCalculator, two_lines_close_right) {
    test({ millimeter_t(70), millimeter_t(100) });
}

TEST(LinePosCalculator, two_lines_far) {
    test({ millimeter_t(-80), millimeter_t(70) });
}