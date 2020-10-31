#include <micro/math/numeric.hpp>
#include <micro/test/utils.hpp>
#include <LinePosCalculator.hpp>

#define PRINT_MEAS false
#include <cmath>

#if PRINT_MEAS
#include <iomanip>
#include <iostream>
#include <string>
#endif // PRINT_MEAS

using namespace micro;

namespace {

constexpr uint32_t NUM_TESTS_PER_SCENARIO = 10000;

constexpr std::pair<uint8_t, uint8_t> SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 0,  255 }, // 0
    { 0,  255 }, // 1
    { 0,  255 }, // 2
    { 0,  255 }, // 3
    { 0,  255 }, // 4
    { 0,  255 }, // 5
    { 0,  255 }, // 6
    { 0,  255 }, // 7
    { 0,  255 }, // 8
    { 0,  255 }, // 9
    { 0,  255 }, // 10
    { 0,  255 }, // 11
    { 0,  255 }, // 12
    { 0,  255 }, // 13
    { 0,  255 }, // 14
    { 0,  255 }, // 15
    { 0,  255 }, // 16
    { 0,  255 }, // 17
    { 0,  255 }, // 18
    { 0,  255 }, // 19
    { 0,  255 }, // 20
    { 0,  255 }, // 21
    { 0,  255 }, // 22
    { 0,  255 }, // 23
    { 0,  255 }, // 24
    { 0,  255 }, // 25
    { 0,  255 }, // 26
    { 0,  255 }, // 27
    { 0,  255 }, // 28
    { 0,  255 }, // 29
    { 0,  255 }, // 30
    { 0,  255 }, // 31
    { 0,  255 }, // 32
    { 0,  255 }, // 33
    { 0,  255 }, // 34
    { 0,  255 }, // 35
    { 0,  255 }, // 36
    { 0,  255 }, // 37
    { 0,  255 }, // 38
    { 0,  255 }, // 39
    { 0,  255 }, // 40
    { 0,  255 }, // 41
    { 0,  255 }, // 42
    { 0,  255 }, // 43
    { 0,  255 }, // 44
    { 0,  255 }, // 45
    { 0,  255 }, // 46
    { 0,  255 }  // 47
};

void createMeasurements(const vec<millimeter_t, Line::MAX_NUM_LINES>& lines, Measurements& meas) {

    static constexpr std::pair<uint8_t, uint8_t> TYPICAL_RANGE = { 20, 170 };
    static constexpr double RAND_WEIGHT = 0.25;
    static constexpr double SIGMA = 1.0;
    static constexpr double MAX_Z_SCORE = 1.0 / (SIGMA * std::sqrt(2 * M_PI));

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        meas[i] = 0;
    }

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        for (millimeter_t linePos : lines) {
            const double z_score = (i - LinePosCalculator::linePosToOptoPos(linePos)) / SIGMA;
            const double value = 1.0 / (SIGMA * std::sqrt(2 * M_PI)) * exp(-0.5 * z_score * z_score);
            const float rand_mul = map<uint32_t, double>(rand() % 10000, 0, 10000, 1 - RAND_WEIGHT, 1 + RAND_WEIGHT);
            const uint8_t incr = map(value / MAX_Z_SCORE, 0.0, 1.0, TYPICAL_RANGE.first, TYPICAL_RANGE.second) * rand_mul;
            meas[i] = std::numeric_limits<uint8_t>::max() - incr > meas[i] ? meas[i] + incr : std::numeric_limits<uint8_t>::max();
        }
    }
}

void test(const vec<millimeter_t, Line::MAX_NUM_LINES>& lines) {
    LinePosCalculator linePosCalculator(SENSOR_LIMITS);
    Measurements measurements;

    for (uint32_t i = 0; i < NUM_TESTS_PER_SCENARIO; ++i) {
        createMeasurements(lines, measurements);

#if PRINT_MEAS
        std::cout << "Test meas:";
        for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
            std::cout << std::setw(3) << std::to_string(measurements[i]);
            if (i < cfg::NUM_SENSORS - 1) {
                std::cout << ",";
            } else {
                std::cout << std::endl;
            }
        }
#endif // PRINT_MEAS

        const LinePositions linePositions = linePosCalculator.calculate(measurements);
        EXPECT_EQ(lines.size(), linePositions.size());
        for (uint8_t i = 0; i < linePositions.size(); ++i) {
            EXPECT_NEAR_UNIT(lines[i], linePositions[i].pos, millimeter_t(4));
            EXPECT_LE(0.65f, linePositions[i].probability);
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