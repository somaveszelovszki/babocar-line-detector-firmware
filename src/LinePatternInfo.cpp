#include <LinePatternInfo.hpp>

#include <micro/container/vector.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/Line.hpp>

#include <LinePatternCalculator.hpp>

using namespace micro;

namespace {

const micro::Lines& pastLines(const LinePatternCalculator::Measurements& measurements) {
    static const micro::Lines EMPTY_LINES{};

    if (measurements.empty()) {
        return EMPTY_LINES;
    }

    constexpr auto PEEK_BACK_DISTANCE = centimeter_t(15);

    const meter_t dist = measurements.back().distance - PEEK_BACK_DISTANCE;

    int32_t startIdx = 1;
    int32_t endIdx = static_cast<int32_t>(measurements.size()) - 1;

    while (startIdx < endIdx - 1) {
        const int32_t i = micro::avg(startIdx, endIdx);
        if (std::next(measurements.rbegin(), i)->distance < dist) {
            endIdx = i;
        } else {
            startIdx = i;
        }
    }

    return std::next(measurements.rbegin(), endIdx)->lines;
}

bool isInJunctionCenter(const Lines& lines) {
    return 1 < lines.size() && micro::areClose(lines);
}

Lines::const_iterator expectedMainLine(const LinePattern& pattern, const Lines& lines, const Sign speedSign) {
    Direction expectedMainLineDir = Direction::CENTER;

    if (LinePattern::LANE_CHANGE == pattern.type) {
        if (Sign::POSITIVE == speedSign) {
            expectedMainLineDir = -pattern.side;
        } else {
            expectedMainLineDir = pattern.side;
        }
    } else {
        if (pattern.dir == speedSign) {
            expectedMainLineDir = pattern.side;
        } else {
            expectedMainLineDir = -pattern.side;
        }
    }

    if (lines.empty()) {
        return lines.end();
    }

    switch (expectedMainLineDir) {
    case Direction::LEFT:
        return lines.begin();

    case Direction::CENTER:
        return std::next(lines.begin(), lines.size() / 2);

    case Direction::RIGHT:
        return std::next(lines.begin(), lines.size() - 1);

    default:
        return lines.end();
    }
}

const micro::vector<LinePatternCalculator::LinePatternInfo, 10> PATTERN_INFO = {
    { // NONE
        [](const micro::Sign&) { return centimeter_t(10); },
        micro::numeric_limits<meter_t>::infinity(),
        [] (const LinePatternCalculator::Measurements&, const LinePattern&, const Lines& lines, const Line&, meter_t, Sign) {
            return 0 == lines.size();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });

            } else if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::BRAKE,       Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // SINGLE_LINE
        [](const micro::Sign&) { return centimeter_t(5); },
        micro::numeric_limits<meter_t>::infinity(),
        [] (const LinePatternCalculator::Measurements&, const LinePattern&, const Lines& lines, const Line&, meter_t, Sign) {
            return 1 == lines.size();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                validPatterns.push_back({ LinePattern::NONE,        Sign::NEUTRAL,  Direction::CENTER });
                validPatterns.push_back({ LinePattern::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER });
                validPatterns.push_back({ LinePattern::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.push_back({ LinePattern::JUNCTION_2,  Sign::NEGATIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::JUNCTION_3,  Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.push_back({ LinePattern::JUNCTION_3,  Sign::NEGATIVE, Direction::CENTER });
                validPatterns.push_back({ LinePattern::JUNCTION_3,  Sign::NEGATIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::LANE_CHANGE, Sign::POSITIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::LANE_CHANGE, Sign::NEGATIVE, Direction::LEFT   });

            } else if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::NONE,       Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::BRAKE,      Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // ACCELERATE
        [](const micro::Sign&) { return centimeter_t(18); },
        centimeter_t(85),
        [] (const LinePatternCalculator::Measurements&, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {

            static const LinePatternDescriptor descriptor = {
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) }
            };

            const auto validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(2.5f));
            return areClose(lines) && std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // BRAKE
        [](const micro::Sign&) { return centimeter_t(12); },
        centimeter_t(350),
        [] (const LinePatternCalculator::Measurements&, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {
            return areClose(lines) && 3 == lines.size();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // LANE_CHANGE
        [](const micro::Sign&) { return centimeter_t(35); },
        centimeter_t(120),
        [] (const LinePatternCalculator::Measurements&, const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, meter_t currentDist, Sign speedSign) {

            static const LinePatternDescriptor descriptor = {
                { 2, centimeter_t(16) },
                { 1, centimeter_t(14) },
                { 2, centimeter_t(14) },
                { 1, centimeter_t(12) },
                { 2, centimeter_t(12) },
                { 1, centimeter_t(10) },
                { 2, centimeter_t(10) },
                { 1, centimeter_t(8)  },
                { 2, centimeter_t(8)  }
            };

            const auto validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(2.5f));

            return micro::areClose(lines)                                                            &&
                   std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end() &&
                   LinePatternCalculator::getMainLine(lines, lastSingleLine) == expectedMainLine(pattern, lines, speedSign);
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                validPatterns.push_back({ LinePattern::NONE,        Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // JUNCTION_1
        [](const micro::Sign& dir) { return centimeter_t(dir == micro::Sign::POSITIVE ? 30 : 4); },
        centimeter_t(80),
        [] (const LinePatternCalculator::Measurements& measurements, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {     

            switch (pattern.dir) {
            case micro::Sign::NEGATIVE:
                return isInJunctionCenter(lines);

            case micro::Sign::POSITIVE:
                return 1 == lines.size();

            default:
                return false;
            }
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_2
        [](const micro::Sign&) { return centimeter_t(8); },
        centimeter_t(80),
        [] (const LinePatternCalculator::Measurements& measurements, const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, meter_t, Sign speedSign) {
            const auto areValidFarLines = [&pattern, &lastSingleLine, &speedSign](const Lines& lines) {
                return 2 == lines.size() && areFar(lines) &&
                    LinePatternCalculator::getMainLine(lines, lastSingleLine) == expectedMainLine(pattern, lines, speedSign);
            };
            

            switch (pattern.dir) {
            case micro::Sign::NEGATIVE:
                return areValidFarLines(lines) ||
                    (2 == lines.size() && isInJunctionCenter(lines) && areValidFarLines(pastLines(measurements)));

            case micro::Sign::POSITIVE:
                return 2 == lines.size();

            default:
                return false;
            }
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_CENTER, Sign::NEUTRAL, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_3
        [](const micro::Sign&) { return centimeter_t(8); },
        centimeter_t(80),
        [] (const LinePatternCalculator::Measurements& measurements, const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, meter_t, Sign speedSign) {
            const auto areValidFarLines = [&pattern, &lastSingleLine, &speedSign](const Lines& lines) {
                return 1 < lines.size() && micro::areFar(lines) &&
                    ((2 == lines.size() && Direction::CENTER == pattern.side) ||
                    LinePatternCalculator::getMainLine(lines, lastSingleLine) == expectedMainLine(pattern, lines, speedSign));
            };
            
            switch (pattern.dir) {
            case micro::Sign::NEGATIVE:
                return areValidFarLines(lines) ||
                    (3 == lines.size() && isInJunctionCenter(lines) && areValidFarLines(pastLines(measurements)));

            case micro::Sign::POSITIVE:
                return 3 == lines.size();

            default:
                return false;
            }
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_CENTER, Sign::NEUTRAL, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_CENTER
        [](const micro::Sign&) { return centimeter_t(4); },
        centimeter_t(100),
        [] (const LinePatternCalculator::Measurements&, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {
            return currentDist - pattern.startDist < centimeter_t(80) && 1 == lines.size() || 4 == lines.size();
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::LinePatterns validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
            }
            return validPatterns;
        }
    }
};

} // namespace

const LinePatternCalculator::LinePatternInfo& getLinePatternInfo(const micro::LinePattern::type_t type) {
    return PATTERN_INFO[static_cast<size_t>(type)];
}
