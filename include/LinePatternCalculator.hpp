#pragma once

#include <micro/container/infinite_buffer.hpp>
#include <micro/utils/LinePattern.hpp>
#include <functional>

class LinePatternCalculator {
public:
    struct StampedLines {
        micro::Lines lines;
        micro::meter_t distance;
    };

    typedef micro::infinite_buffer<StampedLines, 200> measurement_buffer_t;
    typedef micro::infinite_buffer<micro::LinePattern, 200> pattern_buffer_t;
    typedef micro::vec<micro::LinePattern, 10> linePatterns_t;

    struct LinePatternInfo {
        micro::meter_t minValidityLength;
        micro::meter_t maxLength;
        enum {
            USES_HISTORY,
            NO_HISTORY
        } historyDependency;
        std::function<bool(const measurement_buffer_t&, const micro::LinePattern&, const micro::Lines&, uint8_t, micro::meter_t)> isValid;
        std::function<linePatterns_t(const micro::LinePattern&, const micro::linePatternDomain_t)> validNextPatterns;
    };

    LinePatternCalculator()
        : isPatternChangeCheckActive(false)
        , lastSingleLineId(0) {
        this->prevPatterns.push_back({ micro::LinePattern::SINGLE_LINE, micro::Sign::NEUTRAL, micro::Direction::CENTER, micro::meter_t(0) });
    }
    void update(const micro::linePatternDomain_t domain, const micro::Lines& lines, micro::meter_t currentDist);

    const micro::LinePattern& pattern() const {
        return const_cast<LinePatternCalculator*>(this)->currentPattern();
    }

    bool isPending() const {
        return this->isPatternChangeCheckActive;
    }

    static StampedLines peek_back(const measurement_buffer_t& prevMeas, micro::meter_t peekBackDist);

    static micro::Lines::const_iterator getMainLine(const micro::Lines& lines, const uint8_t lastSingleLineId);

private:
    micro::LinePattern& currentPattern() {
        return this->prevPatterns.peek_back(0);
    }

    void changePattern(const micro::LinePattern& newPattern);

    measurement_buffer_t prevMeas;
    pattern_buffer_t prevPatterns;

    bool isPatternChangeCheckActive;
    linePatterns_t possiblePatterns;
    uint8_t lastSingleLineId;
};
