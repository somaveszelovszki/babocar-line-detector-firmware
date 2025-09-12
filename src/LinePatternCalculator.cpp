#include <LinePatternCalculator.hpp>
#include <LinePatternInfo.hpp>

#include <micro/math/unit_utils.hpp>

using namespace micro;

LinePatternDescriptor::LinePatternDescriptor(const std::initializer_list<LineSegment>& pattern)
    : pattern(pattern) {
}

auto LinePatternDescriptor::getValidLines(Sign dir, centimeter_t patternDist,
                                          centimeter_t eps) const -> ValidLinesCount {
    ValidLinesCount validLines;

    const std::pair<const LineSegment*, const LineSegment*> bounds =
        dir != Sign::NEGATIVE ? getBounds(pattern.begin(), pattern.end(), patternDist, eps)
                              : getBounds(pattern.rbegin(), pattern.rend(), patternDist, eps);

    if (bounds.first && bounds.second) {
        const std::pair<uint8_t, uint8_t> validLinesRange = {
            std::min(bounds.first->numLines, bounds.second->numLines),
            std::max(bounds.first->numLines, bounds.second->numLines)};

        for (uint8_t numLines = validLinesRange.first; numLines <= validLinesRange.second;
             ++numLines) {
            validLines.insert(numLines);
        }
    }

    return validLines;
}

void LinePatternCalculator::update(const linePatternDomain_t domain, const Lines& lines,
                                   meter_t currentDist, const Sign speedSign) {
    if (measurements.full()) {
        measurements.pop();
    }
    measurements.push({lines, currentDist});

    if (LinePattern::SINGLE_LINE == pattern_.type && 1 == lines.size()) {
        lastSingleLine = *lines.begin();
    }

    if (nextPatternCandidates_.empty()) {
        const auto& currentPatternInfo = getLinePatternInfo(pattern_.type);

        if (currentDist - pattern_.startDist > currentPatternInfo.maxLength) {
            // under normal circumstances, maxLength should never be exceeded
            changePattern({LinePattern::NONE, Sign::NEUTRAL, Direction::CENTER, currentDist});

        } else if ((pattern_.type == LinePattern::JUNCTION_1 && pattern_.dir == Sign::NEGATIVE) ||
                   !currentPatternInfo.isValid(measurements, pattern_, lines, lastSingleLine,
                                               currentDist, speedSign)) {
            nextPatternCandidates_ = currentPatternInfo.validNextPatterns(pattern_, domain);

            for (LinePattern& pattern : nextPatternCandidates_) {
                pattern.startDist = currentDist;
            }
        }
    }

    for (auto it = nextPatternCandidates_.begin(); it != nextPatternCandidates_.end();) {
        const auto& patternInfo = getLinePatternInfo(it->type);
        if (patternInfo.isValid(measurements, *it, lines, lastSingleLine, currentDist, speedSign)) {
            if (1 == nextPatternCandidates_.size() &&
                currentDist - it->startDist >= patternInfo.minValidityLength) {
                changePattern(*it);
                break;
            }
            ++it;
        } else {
            it = nextPatternCandidates_.erase(it);
        }
    }
}

void LinePatternCalculator::changePattern(const LinePattern& newPattern) {
    pattern_ = newPattern;
    nextPatternCandidates_.clear();
}

Lines::const_iterator LinePatternCalculator::getMainLine(const Lines& lines,
                                                         const micro::Line& lastSingleLine) {
    auto mainLine = micro::findLine(lines, lastSingleLine.id);
    if (mainLine == lines.end()) {
        mainLine = micro::findClosestLine(lines, lastSingleLine.pos);
    }
    return mainLine;
}
