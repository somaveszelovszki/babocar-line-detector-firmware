#include <micro/math/unit_utils.hpp>

#include <LinePatternCalculator.hpp>

using namespace micro;

LinePatternDescriptor::LinePatternDescriptor(const std::initializer_list<LineSegment>& pattern)
    : pattern(pattern) {}

auto LinePatternDescriptor::getValidLines(Sign dir, centimeter_t patternDist, centimeter_t eps) const -> ValidLinesCount {
    ValidLinesCount validLines;

    const std::pair<const LineSegment*, const LineSegment*> bounds = dir != Sign::NEGATIVE ?
        this->getBounds(this->pattern.begin(), this->pattern.end(), patternDist, eps) :
        this->getBounds(this->pattern.rbegin(), this->pattern.rend(), patternDist, eps);

    if (bounds.first && bounds.second) {
        const std::pair<uint8_t, uint8_t> validLinesRange = {
             std::min(bounds.first->numLines, bounds.second->numLines),
             std::max(bounds.first->numLines, bounds.second->numLines)
        };

        for (uint8_t numLines = validLinesRange.first; numLines <= validLinesRange.second; ++numLines) {
            validLines.insert(numLines);
        }
    }

    return validLines;
}

void LinePatternCalculator::update(const linePatternDomain_t domain, const Lines& lines, meter_t currentDist, const Sign speedSign) {

    this->prevMeas.push_back({ lines, currentDist });
    LinePattern& current = this->currentPattern();

    if (LinePattern::SINGLE_LINE == current.type && 1 == lines.size()) {
        this->lastSingleLine = lines[0];
    }

    if (this->isPatternChangeCheckActive) {

        for (linePatterns_t::iterator it = possiblePatterns.begin(); it != possiblePatterns.end();) {
            const LinePatternInfo& patternInfo = PATTERN_INFO.at(it->type);
            if (patternInfo.isValid(this->prevMeas, *it, lines, this->lastSingleLine, currentDist, speedSign)) {
                if (1 == possiblePatterns.size() && currentDist - it->startDist >= patternInfo.minValidityLength) {
                    this->changePattern(*it);
                    break;
                }
                ++it;
            } else {
                it = this->possiblePatterns.erase(it);
            }
        }

        if (this->possiblePatterns.empty()) {
            this->isPatternChangeCheckActive = false;
        }

    } else {
        const LinePatternInfo& currentPatternInfo = PATTERN_INFO.at(this->currentPattern().type);

        if (currentDist - current.startDist > currentPatternInfo.maxLength) {
            // under normal circumstances, maxLength should never be exceeded
            this->changePattern({ LinePattern::NONE, Sign::NEUTRAL, Direction::CENTER, currentDist });

        } else if (!currentPatternInfo.isValid(this->prevMeas, current, lines, this->lastSingleLine, currentDist, speedSign)) {
            this->isPatternChangeCheckActive = true;
            this->possiblePatterns = currentPatternInfo.validNextPatterns(current, domain);

            for (LinePattern& pattern : this->possiblePatterns) {
                pattern.startDist = currentDist;
            }
        }
    }
}

void LinePatternCalculator::changePattern(const LinePattern& newPattern) {
    this->prevPatterns.push_back(newPattern);
    this->isPatternChangeCheckActive = false;
}

Lines::const_iterator LinePatternCalculator::getMainLine(const Lines& lines, const micro::Line& lastSingleLine) {
    Lines::const_iterator mainLine = micro::findLine(lines, lastSingleLine.id);
    if (mainLine == lines.end()) {
        mainLine = micro::findClosestLine(lines, lastSingleLine.pos);
    }
    return mainLine;
}
