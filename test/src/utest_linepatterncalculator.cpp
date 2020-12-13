#include <micro/math/numeric.hpp>
#include <micro/test/utils.hpp>
#include <LinePatternCalculator.hpp>

using namespace micro;

namespace {

typedef vec<Lines, 500> LineDetections;
typedef sorted_map<centimeter_t, LinePattern, 10> LinePatternDetections;

void test(const linePatternDomain_t domain, const LineDetections& lineDetections, const LinePatternDetections& expectedPatternDetections) {
    LinePatternCalculator calc;

    for (uint32_t i = 0; i < lineDetections.size(); ++i) {
        const centimeter_t distance = centimeter_t(i);
        const Lines& lines = lineDetections[i];

        calc.update(domain, lines, distance);

        const LinePattern& expectedPattern = expectedPatternDetections.bounds(distance).first->second;
        const LinePattern& currentPattern  = calc.pattern();

        EXPECT_EQ(expectedPattern.type, currentPattern.type);
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

    const LinePatternDetections& expectedPatternDetections = {
        { centimeter_t(0), { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER } }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatternDetections);
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

    const LinePatternDetections& expectedPatternDetections = {
        { centimeter_t(0),                                               { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER } },
        { PATTERN_INFO.at(LinePattern::type_t::NONE)->minValidityLength, { LinePattern::type_t::NONE,        Sign::NEUTRAL, Direction::CENTER } }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatternDetections);
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

    const LinePatternDetections& expectedPatternDetections = {
        { centimeter_t(0),                                                { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER } },
        { PATTERN_INFO.at(LinePattern::type_t::BRAKE)->minValidityLength, { LinePattern::type_t::BRAKE,       Sign::NEUTRAL, Direction::CENTER } }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatternDetections);
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

    const LinePatternDetections& expectedPatternDetections = {
        { centimeter_t(0),                                                     { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER } },
        { PATTERN_INFO.at(LinePattern::type_t::ACCELERATE)->minValidityLength, { LinePattern::type_t::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER } }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatternDetections);
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

    const LinePatternDetections& expectedPatternDetections = {
        { centimeter_t(0),                                                     { LinePattern::type_t::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER } },
        { PATTERN_INFO.at(LinePattern::type_t::ACCELERATE)->minValidityLength, { LinePattern::type_t::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER } }
    };

    test(linePatternDomain_t::Race, lineDetections, expectedPatternDetections);
}
