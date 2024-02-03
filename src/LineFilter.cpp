#include <micro/container/vector.hpp>
#include <micro/math/numeric.hpp>

#include <LineFilter.hpp>

using namespace micro;

Lines LineFilter::update(const LinePositions& detectedLines, const size_t maxLines) {
    using LinePositionIters = micro::vector<LinePositions::const_iterator, cfg::MAX_NUM_FILTERED_LINES>;
    LinePositionIters unmatchedDetectedLines;
    for (LinePositions::const_iterator it = detectedLines.begin(); it != detectedLines.end(); ++it) {
        unmatchedDetectedLines.push_back(it);
    }

    using FilteredLineIters = micro::vector<FilteredLines::iterator, cfg::MAX_NUM_FILTERED_LINES> ;
    FilteredLineIters unmatchedFilteredLines;
    for (FilteredLines::iterator it = lines_.begin(); it != lines_.end(); ++it) {
        unmatchedFilteredLines.push_back(it);
    }

    // updates estimated positions for all filtered lines
    for (FilteredLine& l : lines_) {
        const millimeter_t current = l.current_raw();

        l.estimated = l.samples.size() >= cfg::LINE_VELO_FILTER_SIZE ?
            current + (current - *std::next(l.samples.rbegin(), cfg::LINE_VELO_FILTER_SIZE - 1)) / cfg::LINE_VELO_FILTER_SIZE :
            current;
    }

    struct posMapping_t {
        LinePositionIters::iterator detectedLine;
        FilteredLineIters::iterator filteredLine;
        millimeter_t diff;
    };

    typedef micro::vector<posMapping_t, Line::MAX_NUM_LINES * cfg::MAX_NUM_FILTERED_LINES> posMappings_t;

    // finds all close position pairs from the current and the previous measurements (expected positions), and updates filtered lines
    while (unmatchedDetectedLines.size() && unmatchedFilteredLines.size()) {

        // maps all previous and current line positions to each other
        posMappings_t posMappings;
        for (LinePositionIters::iterator detectedLine = unmatchedDetectedLines.begin(); detectedLine != unmatchedDetectedLines.end(); ++detectedLine) {
            for (FilteredLineIters::iterator filteredLine = unmatchedFilteredLines.begin(); filteredLine != unmatchedFilteredLines.end(); ++filteredLine) {
                posMappings.push_back({ detectedLine, filteredLine, abs((*detectedLine)->pos - (*filteredLine)->estimated) });
            }
        }

        // finds closest pair in the line position map
        const posMappings_t::iterator closest = std::min_element(posMappings.begin(), posMappings.end(), [] (const posMapping_t& a, const posMapping_t& b) {
            return a.diff < b.diff;
        });

        // will be accepted as valid position pairs of the previous and the current measurement if they are close enough to each other
        if (closest->diff < cfg::MAX_LINE_JUMP) {
            auto& it = *closest->filteredLine;
            if (it->samples.full()) {
                it->samples.pop();
            }
            it->samples.push((*closest->detectedLine)->pos);
            it->increaseCntr();
        } else {
            // no more close pairs found
            break;
        }

        // pair has been handled, removes them from their correspondent list
        unmatchedDetectedLines.erase(closest->detectedLine);
        unmatchedFilteredLines.erase(closest->filteredLine);
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
        // erases lines from the filtered lines list that have not been detected for a given number of measurements
        if (-cfg::LINE_FILTER_HYSTERESIS == it->cntr) {
            it = lines_.erase(it);
        } else {
            // if a line has been in the filtered lines list for at least LINE_FILTER_HYSTERESIS measurements, then it is a valid line
            if (cfg::LINE_FILTER_HYSTERESIS == it->cntr) {
                it->isValidated = true;
            }

            // output list will contain all validated lines from the filtered lines list
            if (it->isValidated && validLines.size() < maxLines) {
                validLines.insert({ it->current(), it->id });
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
        newLine.id = generateNewLineId();
        newLine.cntr = 1;
        newLine.isValidated = false;
        newLine.samples.push(it->pos);
        lines_.insert(newLine);
    }

    return validLines;
}

millimeter_t LineFilter::FilteredLine::current() const {
    const auto size = std::min<size_t>(samples.size(), cfg::LINE_POS_FILTER_WINDOW_SIZE);
    const auto end = std::next(samples.rbegin(), size);
    millimeter_t pos;

    for (auto it = samples.rbegin(); it != end; it++) {
        pos += *it;
    }

    return pos / size;
}

uint8_t LineFilter::generateNewLineId() {
    uint8_t id = 1;
    while (std::find_if(lines_.begin(), lines_.end(), [id] (const FilteredLine& l) { return id == l.id; }) != lines_.end()) { ++id; }
    return id;
}
