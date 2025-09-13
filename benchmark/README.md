# LinePosCalculator Benchmark

This directory contains performance benchmarks for the `LinePosCalculator` class.

## Building

To build the benchmarks, configure CMake with the `BUILD_BENCHMARKS` option:

```bash
mkdir build
cd build
cmake .. -DBUILD_BENCHMARKS=ON
ninja line_detector_benchmark  # or make line_detector_benchmark if using Make
```

## Running

Run the benchmark executable:

```bash
./firmware/line-detector/benchmark/line_detector_benchmark
```

### Benchmark Options

You can customize the benchmark execution with various options:

```bash
# Run for a minimum time per benchmark
./line_detector_benchmark --benchmark_min_time=1s

# Filter specific benchmarks
./line_detector_benchmark --benchmark_filter="SingleLine.*"

# Output results in JSON format
./line_detector_benchmark --benchmark_format=json

# Show help for all options
./line_detector_benchmark --help
```

## Available Benchmarks

- **BM_LineCalculationPipeline**: Complete line calculation pipeline as used in LineCalcTask
  - Includes LinePosCalculator, LineFilter, and LinePatternCalculator
  - Tests single line case with positive speed sign and FRONT panel version
  - Simulates the full processing chain: sensor measurements → line positions → filtered lines → line pattern

## Understanding Results

The benchmark output shows:

- **Time**: Wall clock time per iteration
- **CPU**: CPU time per iteration
- **Iterations**: Number of iterations run

Lower times indicate better performance.
