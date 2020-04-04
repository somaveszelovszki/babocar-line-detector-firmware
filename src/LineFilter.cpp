#include <micro/container/map.hpp>
#include <micro/math/numeric.hpp>
#include <LineFilter.hpp>

using namespace micro;

Lines LineFilter::update(const linePositions_t& detectedLines) {

    typedef vec<linePositions_t::const_iterator, cfg::MAX_NUM_FILTERED_LINES> linePositionIterators_t;
    linePositionIterators_t unmatchedDetectedLines;
    for (linePositions_t::const_iterator it = detectedLines.begin(); it != detectedLines.end(); ++it) {
        unmatchedDetectedLines.push_back(it);
    }

    typedef vec<filteredLines_t::iterator, cfg::MAX_NUM_FILTERED_LINES> filteredLineIterators_t;
    filteredLineIterators_t unmatchedFilteredLines;
    for (filteredLines_t::iterator it = this->lines_.begin(); it != this->lines_.end(); ++it) {
        unmatchedFilteredLines.push_back(it);
    }

    // updates estimated positions for all filtered lines
    for (filteredLine_t& l : this->lines_) {
        l.estimated = l.samples.size() > cfg::LINE_FILTER_VELO_PEEK_BACK ?
            l.current() + (l.current() - l.samples.peek_back(cfg::LINE_FILTER_VELO_PEEK_BACK)) / cfg::LINE_FILTER_VELO_PEEK_BACK :
            l.current();
    }

    struct posMapping_t {
        linePositionIterators_t::iterator detectedLine;
        filteredLineIterators_t::iterator filteredLine;
        millimeter_t diff;
    };

    typedef vec<posMapping_t, cfg::MAX_NUM_LINES * cfg::MAX_NUM_FILTERED_LINES> posMappings_t;

    // finds all close position pairs from the current and the previous measurements (expected positions), and updates filtered lines
    while (unmatchedDetectedLines.size() && unmatchedFilteredLines.size()) {

        // maps all previous and current line positions to each other
        posMappings_t posMappings;
        for (linePositionIterators_t::iterator detectedLine = unmatchedDetectedLines.begin(); detectedLine != unmatchedDetectedLines.end(); ++detectedLine) {
            for (filteredLineIterators_t::iterator filteredLine = unmatchedFilteredLines.begin(); filteredLine != unmatchedFilteredLines.end(); ++filteredLine) {
                posMappings.push_back({ detectedLine, filteredLine, abs((*detectedLine)->pos - (*filteredLine)->estimated) });
            }
        }

        // finds closest pair in the line position map
        const posMappings_t::iterator closest = std::min_element(posMappings.begin(), posMappings.end(), [] (const posMapping_t& a, const posMapping_t& b) {
            return a.diff < b.diff;
        });

        // will be accepted as valid position pairs of the previous and the current measurement if they are close enough to each other
        if (closest->diff < cfg::MAX_LINE_JUMP) {
            (*closest->filteredLine)->samples.push_back((*closest->detectedLine)->pos);
            (*closest->filteredLine)->increaseCntr();
        } else {
            // no more close pairs found
            break;
        }

        // pair has been handled, removes them from their correspondent list
        unmatchedDetectedLines.erase(closest->detectedLine);
        unmatchedFilteredLines.erase(closest->filteredLine);
    }

    // decreases counters for unmatched previous lines
    for (filteredLines_t::iterator it : unmatchedFilteredLines) {
        it->decreaseCntr();
    }

    // erases lines from the filtered lines list that have not been detected for a given number of measurements
    for (filteredLines_t::const_iterator it = this->lines_.begin(); it != this->lines_.end();) {
        if (-cfg::LINE_FILTER_HYSTERESIS == it->cntr) {
            it = this->lines_.erase(it);
        } else {
            ++it;
        }
    }

    // if a line has been in the filtered lines list for at least LINE_FILTER_HYSTERESIS measurements, then it is a valid line
    for (filteredLine_t& l : this->lines_) {
        if (cfg::LINE_FILTER_HYSTERESIS == l.cntr) {
            l.isValidated = true;
        }
    }

    // added unmatched detected lines to the filtered lines list
    for (linePositions_t::const_iterator it : unmatchedDetectedLines) {
        if (it->probability >= cfg::MIN_LINE_APPEAR_PROBABILITY) {
            filteredLine_t newLine;
            newLine.id = this->generateNewLineId();
            newLine.cntr = 1;
            newLine.isValidated = false;
            newLine.samples.push_back(it->pos);
            this->lines_.insert(newLine);
        }
    }

    // output list will contain all validated lines from the filtered lines list
    Lines trackedLines;
    for (const filteredLine_t& l : this->lines_) {
        if (l.isValidated) {
            trackedLines.insert({ l.current(), l.id });
        }
    }

    return trackedLines;
}

uint8_t LineFilter::generateNewLineId() {
    uint8_t id = 1;
    while (std::find_if(this->lines_.begin(), this->lines_.end(), [id] (const filteredLine_t& l) { return id == l.id; }) != this->lines_.end()) { ++id; }
    return id;
}
