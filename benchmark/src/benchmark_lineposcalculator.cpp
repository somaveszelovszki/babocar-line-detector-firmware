#include <cmath>
#include <cstdlib>

#include <LineFilter.hpp>
#include <LinePatternCalculator.hpp>
#include <LinePosCalculator.hpp>
#include <random>

#include <micro/container/vector.hpp>
#include <micro/math/numeric.hpp>
#include <micro/utils/LinePattern.hpp>

#include <benchmark/benchmark.h>

using namespace micro;

namespace {

// Helper function to create realistic sensor measurements
void createMeasurements(const micro::vector<millimeter_t, Line::MAX_NUM_LINES>& lines,
                        Measurements& meas) {
    static const double RANDOM_WEIGHT = 0.25;
    static const double SIGMA         = 1.0;
    static const double MAX_Z_SCORE   = 1.0 / (SIGMA * std::sqrt(2 * M_PI));

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        meas[i] = 0;
    }

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        for (millimeter_t linePos : lines) {
            const double z_score = (i - LinePosCalculator::linePosToOptoPos(linePos)) / SIGMA;
            const double value =
                1.0 / (SIGMA * std::sqrt(2 * M_PI)) * exp(-0.5 * z_score * z_score);

            // Simple linear interpolation for random multiplier
            const uint32_t rand_val = rand() % 10000;
            const double rand_norm  = static_cast<double>(rand_val) / 10000.0;
            const float random_mul  = (1 - RANDOM_WEIGHT) + rand_norm * (2 * RANDOM_WEIGHT);

            // Simple linear interpolation and clamping for increment
            const double norm_value   = value / MAX_Z_SCORE;
            const double scaled_value = norm_value * 255.0 * random_mul;
            const int32_t incr_raw    = static_cast<int32_t>(scaled_value);
            const uint8_t incr =
                (incr_raw < 0) ? 0 : ((incr_raw > 255) ? 255 : static_cast<uint8_t>(incr_raw));

            meas[i] = std::numeric_limits<uint8_t>::max() - incr > meas[i]
                          ? meas[i] + incr
                          : std::numeric_limits<uint8_t>::max();
        }
    }
}

} // namespace

// Benchmark full line calculation pipeline as used in LineCalcTask
static void BM_LineCalculationPipeline(benchmark::State& state) {
    // Initialize components as in LineCalcTask
    LinePosCalculator linePosCalc(false); // with calibration disabled
    LineFilter lineFilter;
    LinePatternCalculator linePatternCalc;

    // Test parameters as specified:
    // - Single line (maxLines = 1)
    // - Speed sign positive (sgn(speed) = +1)
    // - Panel version FRONT
    const auto maxLines              = 1; // Single line case
    const linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
    const meter_t distance           = meter_t(10.0f); // Sample distance
    const Sign speedSign             = Sign::POSITIVE; // Positive speed sign

    // Pre-generate measurements to avoid timing overhead
    Measurements measurements;
    micro::vector<millimeter_t, Line::MAX_NUM_LINES> testLines = {
        millimeter_t(0)}; // Single line at center
    createMeasurements(testLines, measurements);

    for (auto _ : state) {
        auto linePositions = linePosCalc.calculate(measurements, maxLines);
        auto lines         = lineFilter.update(linePositions, maxLines);
        linePatternCalc.update(domain, lines, distance, speedSign);
        auto pattern = linePatternCalc.pattern();

        // Prevent optimization of results
        benchmark::DoNotOptimize(linePositions);
        benchmark::DoNotOptimize(lines);
        benchmark::DoNotOptimize(pattern);
    }
}
BENCHMARK(BM_LineCalculationPipeline);
