#pragma once

#include <functional>

#include <etl/vector.h>

#include <micro/container/infinite_buffer.hpp>
#include <micro/container/map.hpp>
#include <micro/utils/LinePattern.hpp>

class LinePatternDescriptor {
public:
    struct LineSegment {
        uint8_t numLines;
        micro::centimeter_t length;
    };

    typedef micro::set<uint8_t, micro::Line::MAX_NUM_LINES + 1> ValidLinesCount;

    LinePatternDescriptor(const std::initializer_list<LineSegment>& pattern);

    ValidLinesCount getValidLines(micro::Sign dir, micro::centimeter_t patternDist, micro::centimeter_t eps) const;

private:
    template <typename Iter>
    std::pair<const LineSegment*, const LineSegment*> getBounds(Iter patternBegin, Iter patternEnd, micro::centimeter_t patternDist, micro::centimeter_t eps) const {
        std::pair<const LineSegment*, const LineSegment*> bounds = { nullptr, nullptr };

        micro::centimeter_t d(0);

        for (Iter it = patternBegin; it != patternEnd; ++it) {

            if (micro::isBtw(patternDist, d - eps, d + eps)) {
                bounds.first  = micro::to_raw_pointer(it == patternBegin ? it : std::prev(it));
                bounds.second = micro::to_raw_pointer(it);

            } else if (micro::isBtw(patternDist, d + eps, d + it->length - eps)) {
                bounds.first  = micro::to_raw_pointer(it);
                bounds.second = micro::to_raw_pointer(it);

            } else if (micro::isBtw(patternDist, d + it->length - eps, d + it->length + eps)) {
                bounds.first  = micro::to_raw_pointer(it);
                bounds.second = micro::to_raw_pointer(std::next(it) == patternEnd ? it : std::next(it));
            }

            if (bounds.first && bounds.second) {
                break;
            }

            d += it->length;
        }

        return bounds;
    }

    etl::vector<LineSegment, 20> pattern;
};

class LinePatternCalculator {
public:
    struct StampedLines {
        micro::Lines lines;
        micro::meter_t distance;
    };

    typedef micro::infinite_buffer<StampedLines, 1000> measurement_buffer_t;
    typedef micro::infinite_buffer<micro::LinePattern, 200> pattern_buffer_t;
    typedef etl::vector<micro::LinePattern, 20> linePatterns_t;

    struct LinePatternInfo {
        micro::meter_t minValidityLength;
        micro::meter_t maxLength;

        std::function<bool(const measurement_buffer_t&, const micro::LinePattern&, const micro::Lines&, const micro::Line&, micro::meter_t, micro::Sign)> isValid;
        std::function<linePatterns_t(const micro::LinePattern&, const micro::linePatternDomain_t)> validNextPatterns;
    };

    LinePatternCalculator()
        : isPatternChangeCheckActive(false) {
        this->prevPatterns.push_back({ micro::LinePattern::SINGLE_LINE, micro::Sign::NEUTRAL, micro::Direction::CENTER, micro::meter_t(0) });
    }
    void update(const micro::linePatternDomain_t domain, const micro::Lines& lines, micro::meter_t currentDist, const micro::Sign speedSign);

    const micro::LinePattern& pattern() const {
        return const_cast<LinePatternCalculator*>(this)->currentPattern();
    }

    bool isPending() const {
        return this->isPatternChangeCheckActive;
    }

    static micro::Lines::const_iterator getMainLine(const micro::Lines& lines, const micro::Line& lastSingleLine);

private:
    micro::LinePattern& currentPattern() {
        return this->prevPatterns.peek_back(0);
    }

    void changePattern(const micro::LinePattern& newPattern);

    measurement_buffer_t prevMeas;
    pattern_buffer_t prevPatterns;

    bool isPatternChangeCheckActive;
    linePatterns_t possiblePatterns;
    micro::Line lastSingleLine;
};

extern const micro::sorted_map<micro::LinePattern::type_t, LinePatternCalculator::LinePatternInfo, 10> PATTERN_INFO;
