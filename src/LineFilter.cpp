#include <LineFilter.hpp>

#include <micro/container/vector.hpp>
#include <micro/math/numeric.hpp>

using namespace micro;

Lines LineFilter::update(const LinePositions& detectedLines, const size_t maxLines) {
    using LinePositionIters =
        micro::vector<LinePositions::const_iterator, cfg::MAX_NUM_FILTERED_LINES>;
    LinePositionIters unmatchedDetectedLines;
    for (LinePositions::const_iterator it = detectedLines.begin(); it != detectedLines.end();
         ++it) {
        unmatchedDetectedLines.push_back(it);
    }

    using FilteredLineIters = micro::vector<FilteredLines::iterator, cfg::MAX_NUM_FILTERED_LINES>;
    FilteredLineIters unmatchedFilteredLines;
    for (FilteredLines::iterator it = lines_.begin(); it != lines_.end(); ++it) {
        unmatchedFilteredLines.push_back(it);
        auto& l = *it;

        // updates estimated position for filtered line
        const millimeter_t current = l.current_raw();

        l.estimated = l.samples.size() >= cfg::LINE_VELO_FILTER_SIZE
                          ? current + (current - *std::next(l.samples.rbegin(),
                                                            cfg::LINE_VELO_FILTER_SIZE - 1)) /
                                          cfg::LINE_VELO_FILTER_SIZE
                          : current;
    }

    // finds all close position pairs from the current and the previous measurements (expected
    // positions), and updates filtered lines
    while (unmatchedDetectedLines.size() && unmatchedFilteredLines.size()) {
        // find the closest pair directly without storing all pairs
        millimeter_t minDiff = millimeter_t(std::numeric_limits<float>::max());
        LinePositionIters::iterator closestDetected;
        FilteredLineIters::iterator closestFiltered;

        for (LinePositionIters::iterator detectedLine = unmatchedDetectedLines.begin();
             detectedLine != unmatchedDetectedLines.end(); ++detectedLine) {
            for (FilteredLineIters::iterator filteredLine = unmatchedFilteredLines.begin();
                 filteredLine != unmatchedFilteredLines.end(); ++filteredLine) {
                const millimeter_t diff = abs((*detectedLine)->pos - (*filteredLine)->estimated);
                if (diff < minDiff) {
                    minDiff         = diff;
                    closestDetected = detectedLine;
                    closestFiltered = filteredLine;
                }
            }
        }

        // will be accepted as valid position pairs of the previous and the current measurement if
        // they are close enough to each other
        if (minDiff < cfg::MAX_LINE_JUMP) {
            auto& it = *closestFiltered;
            if (it->samples.full()) {
                it->samples.pop();
            }
            it->samples.push((*closestDetected)->pos);
            it->increaseCntr();
        } else {
            // no more close pairs found
            break;
        }

        // pair has been handled, removes them from their correspondent list
        unmatchedDetectedLines.erase(closestDetected);
        unmatchedFilteredLines.erase(closestFiltered);
    }

    // decreases counters for unmatched previous lines
    for (FilteredLines::iterator it : unmatchedFilteredLines) {
        it->decreaseCntr();
        if (it->samples.full()) {
            it->samples.pop();
        }
        it->samples.push(it->estimated);
    }

    Lines validLines;

    for (auto it = lines_.begin(); it != lines_.end();) {
        // erases lines from the filtered lines list that have not been detected for a given number
        // of measurements
        if (-cfg::LINE_FILTER_HYSTERESIS == it->cntr) {
            it = lines_.erase(it);
        } else {
            // if a line has been in the filtered lines list for at least LINE_FILTER_HYSTERESIS
            // measurements, then it is a valid line
            if (cfg::LINE_FILTER_HYSTERESIS == it->cntr) {
                it->isValidated = true;
            }

            // output list will contain all validated lines from the filtered lines list
            if (it->isValidated && validLines.size() < maxLines) {
                validLines.insert({it->current(), it->id});
            }

            ++it;
        }
    }

    // added unmatched detected lines to the filtered lines list
    for (LinePositions::const_iterator it : unmatchedDetectedLines) {
        if (lines_.full()) {
            break;
        }

        FilteredLine newLine;
        newLine.id          = generateNewLineId();
        newLine.cntr        = 1;
        newLine.isValidated = false;
        newLine.samples.push(it->pos);
        lines_.insert(newLine);
    }

    return validLines;
}

millimeter_t LineFilter::FilteredLine::current() const {
    const auto size = std::min<size_t>(samples.size(), cfg::LINE_POS_FILTER_WINDOW_SIZE);
    const auto end  = std::next(samples.rbegin(), size);
    millimeter_t pos;

    for (auto it = samples.rbegin(); it != end; it++) {
        pos += *it;
    }

    return pos / size;
}

uint8_t LineFilter::generateNewLineId() {
    uint8_t id = 1;
    while (std::find_if(lines_.begin(), lines_.end(),
                        [id](const FilteredLine& l) { return id == l.id; }) != lines_.end()) {
        ++id;
    }
    return id;
}
