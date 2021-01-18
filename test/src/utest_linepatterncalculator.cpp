#include <micro/math/numeric.hpp>
#include <micro/test/utils.hpp>
#include <LinePatternCalculator.hpp>

using namespace micro;

namespace {

typedef vec<Lines, 500> LineDetections;
typedef vec<LinePattern, 10> LinePatterns;

void test(const linePatternDomain_t domain, const LineDetections& lineDetections, const LinePatterns& expectedPatterns) {
    LinePatternCalculator calc;
    LinePatterns patterns;

    for (uint32_t i = 0; i < lineDetections.size(); ++i) {
        const centimeter_t distance = centimeter_t(i);
        const Lines& lines = lineDetections[i];

        calc.update(domain, lines, distance);

        const LinePattern& currentPattern = calc.pattern();
        if (patterns.empty() || *patterns.back() != currentPattern) {
            patterns.push_back(currentPattern);
        }
    }

    ASSERT_EQ(expectedPatterns.size(), patterns.size());
    for (uint32_t i = 0; i < patterns.size(); ++i) {
        EXPECT_EQ(expectedPatterns[i], patterns[i]);
    }
}

} // namespace

TEST(LinePatternCalculator, SINGLE_LINE) {

    const LineDetections lineDetections = {
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } }
    };

    const LinePatterns& expectedPatterns = {
        { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatterns);
}

TEST(LinePatternCalculator, NONE) {

    const LineDetections lineDetections = {
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {}
    };

    const LinePatterns& expectedPatterns = {
        { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER },
        { LinePattern::type_t::NONE,        Sign::NEUTRAL, Direction::CENTER }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatterns);
}

TEST(LinePatternCalculator, BRAKE) {

    const LineDetections lineDetections = {
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } }
    };

    const LinePatterns& expectedPatterns = {
        { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER },
        { LinePattern::type_t::BRAKE,       Sign::NEUTRAL, Direction::CENTER }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatterns);
}

TEST(LinePatternCalculator, ACCELERATION) {

    const LineDetections lineDetections = {
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } }
    };

    const LinePatterns& expectedPatterns = {
        { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER },
        { LinePattern::type_t::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatterns);
}

TEST(LinePatternCalculator, ACCELERATION_noisy) {

    const LineDetections lineDetections = {
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(-38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) } }
    };

    const LinePatterns& expectedPatterns = {
        { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER },
        { LinePattern::type_t::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatterns);
}

TEST(LinePatternCalculator, JUNCTION_2_NEGATIVE_LEFT_TO_JUNCTION_2_POSITIVE_RIGHT) {

    const LineDetections lineDetections = {
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(0) } },
        { { millimeter_t(-83) }, { millimeter_t(0) } },
        { { millimeter_t(-79) }, { millimeter_t(0) } },
        { { millimeter_t(-76) }, { millimeter_t(0) } },
        { { millimeter_t(-73) }, { millimeter_t(0) } },
        { { millimeter_t(-70) }, { millimeter_t(0) } },
        { { millimeter_t(-68) }, { millimeter_t(0) } },
        { { millimeter_t(-66) }, { millimeter_t(0) } },
        { { millimeter_t(-64) }, { millimeter_t(0) } },
        { { millimeter_t(-62) }, { millimeter_t(0) } },
        { { millimeter_t(-60) }, { millimeter_t(0) } },
        { { millimeter_t(-58) }, { millimeter_t(0) } },
        { { millimeter_t(-57) }, { millimeter_t(0) } },
        { { millimeter_t(-56) }, { millimeter_t(0) } },
        { { millimeter_t(-55) }, { millimeter_t(0) } },
        { { millimeter_t(-54) }, { millimeter_t(0) } },
        { { millimeter_t(-53) }, { millimeter_t(0) } },
        { { millimeter_t(-52) }, { millimeter_t(0) } },
        { { millimeter_t(-51) }, { millimeter_t(0) } },
        { { millimeter_t(-50) }, { millimeter_t(0) } },
        { { millimeter_t(-49) }, { millimeter_t(0) } },
        { { millimeter_t(-48) }, { millimeter_t(0) } },
        { { millimeter_t(-47) }, { millimeter_t(0) } },
        { { millimeter_t(-46) }, { millimeter_t(0) } },
        { { millimeter_t(-45) }, { millimeter_t(0) } },
        { { millimeter_t(-44) }, { millimeter_t(0) } },
        { { millimeter_t(-42) }, { millimeter_t(0) } },
        { { millimeter_t(-42) }, { millimeter_t(0) } },
        { { millimeter_t(-41) }, { millimeter_t(0) } },
        { { millimeter_t(-40) }, { millimeter_t(0) } },
        { { millimeter_t(-39) }, { millimeter_t(0) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(-38) }, { millimeter_t(0) }, { millimeter_t(38) } },
        { { millimeter_t(0) }, { millimeter_t(39) } },
        { { millimeter_t(0) }, { millimeter_t(40) } },
        { { millimeter_t(0) }, { millimeter_t(41) } },
        { { millimeter_t(0) }, { millimeter_t(42) } },
        { { millimeter_t(0) }, { millimeter_t(43) } },
        { { millimeter_t(0) }, { millimeter_t(44) } },
        { { millimeter_t(0) }, { millimeter_t(45) } },
        { { millimeter_t(0) }, { millimeter_t(46) } },
        { { millimeter_t(0) }, { millimeter_t(47) } },
        { { millimeter_t(0) }, { millimeter_t(48) } },
        { { millimeter_t(0) }, { millimeter_t(49) } },
        { { millimeter_t(0) }, { millimeter_t(50) } },
        { { millimeter_t(0) }, { millimeter_t(51) } },
        { { millimeter_t(0) }, { millimeter_t(52) } },
        { { millimeter_t(0) }, { millimeter_t(53) } },
        { { millimeter_t(0) }, { millimeter_t(54) } },
        { { millimeter_t(0) }, { millimeter_t(55) } },
        { { millimeter_t(0) }, { millimeter_t(56) } },
        { { millimeter_t(0) }, { millimeter_t(57) } },
        { { millimeter_t(0) }, { millimeter_t(58) } },
        { { millimeter_t(0) }, { millimeter_t(60) } },
        { { millimeter_t(0) }, { millimeter_t(62) } },
        { { millimeter_t(0) }, { millimeter_t(64) } },
        { { millimeter_t(0) }, { millimeter_t(66) } },
        { { millimeter_t(0) }, { millimeter_t(68) } },
        { { millimeter_t(0) }, { millimeter_t(70) } },
        { { millimeter_t(0) }, { millimeter_t(73) } },
        { { millimeter_t(0) }, { millimeter_t(76) } },
        { { millimeter_t(0) }, { millimeter_t(79) } },
        { { millimeter_t(0) }, { millimeter_t(83) } }
    };

    const LinePatterns& expectedPatterns = {
        { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER },
        { LinePattern::type_t::JUNCTION_2,  Sign::NEGATIVE, Direction::RIGHT  },
        { LinePattern::type_t::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  }
    };

    test(linePatternDomain_t::Labyrinth, lineDetections, expectedPatterns);
}