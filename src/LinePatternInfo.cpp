#include <LinePatternCalculator.hpp>

using namespace micro;

namespace {

LinePatternCalculator::StampedLines peek_back(const LinePatternCalculator::measurement_buffer_t& prevMeas, meter_t peekBackDist) {
    const meter_t dist = prevMeas.peek_back(0).distance - peekBackDist;

    int32_t startIdx = 1;
    int32_t endIdx = prevMeas.size() - 1;

    while (startIdx < endIdx - 1) {
        const int32_t i = avg(startIdx, endIdx);
        if (prevMeas.peek_back(i).distance < dist) {
            endIdx = i;
        } else {
            startIdx = i;
        }
    }

    return prevMeas.peek_back(endIdx);
}

bool isInJunctionCenter(const Lines& lines) {
    return 1 < lines.size() && micro::areClose(lines);
}

Lines::const_iterator expectedMainLine(const LinePattern& pattern, const Lines& lines, const Sign speedSign) {
    Direction expectedMainLineDir = Direction::CENTER;

    if (pattern.dir == speedSign) {
        expectedMainLineDir = pattern.side;
    } else {
        expectedMainLineDir = -pattern.side;
    }

    Lines::const_iterator line = lines.end();

    switch (expectedMainLineDir) {
    case Direction::LEFT:
        line = lines.begin();
        break;

    case Direction::CENTER:
        line = std::next(lines.begin(), lines.size() / 2);
        break;

    case Direction::RIGHT:
        line = lines.back();
        break;
    }

    return line;
}

bool areValidNegativeFarLines_JUNCTION_2(const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, const Sign speedSign) {
    bool valid = false;
    if (2 == lines.size() && micro::areFar(lines)) {
        valid = LinePatternCalculator::getMainLine(lines, lastSingleLine) == expectedMainLine(pattern, lines, speedSign);
    }
    return valid;
};

bool areValidNegativeFarLines_JUNCTION_3(const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, const Sign speedSign) {
    bool valid = false;
    if (1 < lines.size() && micro::areFar(lines)) {
        if (2 == lines.size() && Direction::CENTER == pattern.side) {
            // when only 2 lines are detected, center junction is valid
            valid = true;
        } else {
            valid = LinePatternCalculator::getMainLine(lines, lastSingleLine) == expectedMainLine(pattern, lines, speedSign);
        }
    }
    return valid;
};

} // namespace

const sorted_map<LinePattern::type_t, LinePatternCalculator::LinePatternInfo, 10> PATTERN_INFO = {
    { LinePattern::NONE, {
        centimeter_t(10),
        micro::numeric_limits<meter_t>::infinity(),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern&, const Lines& lines, const Line&, meter_t, Sign) {
            return 0 == lines.size();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });

            } else if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::BRAKE,       Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    } },
    { LinePattern::SINGLE_LINE, {
        centimeter_t(5),
        micro::numeric_limits<meter_t>::infinity(),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines& lines, const Line&, meter_t, Sign) {
            return 1 == lines.size();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
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
    } },
    { LinePattern::ACCELERATE, {
        centimeter_t(18),
        centimeter_t(85),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {

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

            LinePatternDescriptor::ValidLinesCount validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(2.5f));

            return areClose(lines) && std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    } },
    { LinePattern::BRAKE, {
        centimeter_t(12),
        centimeter_t(350),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {
            return areClose(lines) && 3 == lines.size();
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Race == domain) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    } },
    { LinePattern::LANE_CHANGE, {
        centimeter_t(30),
        centimeter_t(120),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, meter_t currentDist, Sign) {

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

            bool valid = false;
            const LinePatternDescriptor::ValidLinesCount validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(2.5f));

            if (std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end() && micro::areClose(lines)) {
                const Lines::const_iterator mainLine = LinePatternCalculator::getMainLine(lines, lastSingleLine);

                if (Direction::LEFT == pattern.side) {
                    valid = mainLine == lines.back();
                } else if (Direction::RIGHT == pattern.side) {
                    valid = mainLine == lines.begin();
                }
            }

            return valid;
        },
        [] (const LinePattern&, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                validPatterns.push_back({ LinePattern::NONE,        Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    } },
    { LinePattern::JUNCTION_1, {
        centimeter_t(4),
        centimeter_t(130),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, const Line&, meter_t currentDist, Sign) {
            bool valid = false;

            if (Sign::POSITIVE == pattern.dir) {
                const Lines pastLines = peek_back(prevMeas, centimeter_t(8)).lines;
                if (isInJunctionCenter(lines)) {
                    valid = true;
                } else if (1 == lines.size() && isInJunctionCenter(pastLines)) {
                    valid = true;
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                const Lines pastLines = peek_back(prevMeas, centimeter_t(25)).lines;
                if (isInJunctionCenter(lines) && 1 == pastLines.size()) {
                    valid = true;
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    } },
    { LinePattern::JUNCTION_2, {
        centimeter_t(8),
        centimeter_t(130),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, meter_t, Sign speedSign) {
            bool valid = false;
            const Lines pastLines = peek_back(prevMeas, centimeter_t(15)).lines;

            if (Sign::POSITIVE == pattern.dir) {
                if (isInJunctionCenter(lines)) {
                    valid = true;
                } else if (2 == lines.size() && micro::areFar(lines) && isInJunctionCenter(pastLines)) {
                    valid = true;
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (2 == lines.size() && areValidNegativeFarLines_JUNCTION_2(pattern, lines, lastSingleLine, speedSign)) {
                    valid = true;
                } else if (isInJunctionCenter(lines) && 2 == pastLines.size() && areValidNegativeFarLines_JUNCTION_2(pattern, pastLines, lastSingleLine, speedSign)) {
                    valid = true;
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    } },
    { LinePattern::JUNCTION_3, {
        centimeter_t(8),
        centimeter_t(130),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, const Line& lastSingleLine, meter_t, Sign speedSign) {
            bool valid = false;
            const Lines pastLines = peek_back(prevMeas, centimeter_t(15)).lines;

            if (Sign::POSITIVE == pattern.dir) {
                if (isInJunctionCenter(lines)) {
                    valid = true;
                } else if (3 == lines.size() && micro::areFar(lines) && isInJunctionCenter(pastLines)) {
                    valid = true;
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (areValidNegativeFarLines_JUNCTION_3(pattern, lines, lastSingleLine, speedSign)) {
                    valid = true;
                } else if (isInJunctionCenter(lines) && 3 == pastLines.size() && areValidNegativeFarLines_JUNCTION_3(pattern, pastLines, lastSingleLine, speedSign)) {
                    valid = true;
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const linePatternDomain_t domain) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (linePatternDomain_t::Labyrinth == domain) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    } }
};
