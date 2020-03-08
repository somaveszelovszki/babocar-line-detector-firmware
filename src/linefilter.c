#include "linefilter.h"
#include "config.h"

#include <micro/math/numeric.h>

#include <stdlib.h>

static void add_line(lineFilter_t *lineFilter, const line_t *line) {
    if (lineFilter->numLines < MAX_NUM_FILTERED_LINES) {

        filteredLine_t *next = &lineFilter->values[lineFilter->numLines];

        if (lineFilter->numLines > 0) {

            // performs ordered insert: shifts all larger values to the right
            for (uint8_t i = lineFilter->numLines - 1; i <= 0; --i) {
                filteredLine_t *fl = &lineFilter->values[i];
                if (fl->current.pos_mm > line->pos_mm) {
                    *next = *fl;
                } else {
                    break;
                }
                next = fl;
            }
        }

        next->current.pos_mm = next->prev.pos_mm = line->pos_mm;
        next->current.id = (lineFilter->lastLineIdx = 255 == lineFilter->lastLineIdx ? 1 : lineFilter->lastLineIdx + 1);
        next->cntr = 1;

        ++lineFilter->numLines;
    }
}

static void remove_line(lineFilter_t *lineFilter, uint8_t idx) {
    for (uint8_t i = idx; i < lineFilter->numLines - 1; ++i) {
        lineFilter->values[i] = lineFilter->values[i + 1];
    }
    --lineFilter->numLines;
}

void linefilter_initialize(lineFilter_t *lineFilter) {
    lineFilter->numLines = 0;
    lineFilter->lastLineIdx = 0;
}

void linefilter_apply(lineFilter_t *lineFilter, const lines_t *detectedLines, trackedLines_t *filteredLines) {

    // iterates through current lines, updates counters for existing ones and adds new lines to the list
    for (uint8_t i = 0; i < detectedLines->numLines; ++i) {
        const line_t *line = &detectedLines->values[i];

        //finds nearest line in the previous line set (if present)
        uint8_t min_idx = 255;
        int32_t min_dist = -1;
        for (uint8_t j = 0; j < lineFilter->numLines; ++j) {
            const int32_t dist = abs((int32_t)line->pos_mm - (int32_t)lineFilter->values[j].current.pos_mm);
            if (min_dist == -1 || dist <= min_dist) {
                min_dist = dist;
                min_idx = j;
            }
        }

        if (min_dist == -1 || min_dist > MAX_LINE_JUMP_MM) {    // line is NOT present in the previous line set
            add_line(lineFilter, line);
        } else {    // line has been found in the previous line set
            filteredLine_t *fl = &lineFilter->values[min_idx];
            fl->current.pos_mm = line->pos_mm;
            fl->cntr = (fl->cntr < 0 || fl->cntr == MIN_LINE_SAMPLE_APPEAR) ? MIN_LINE_SAMPLE_APPEAR : fl->cntr + 1;
        }
    }

    // iterates through previous lines, updates counters and removes lines that have disappeared
    for (uint8_t i = 0; i < lineFilter->numLines; ++i) {
        filteredLine_t *fl= &lineFilter->values[i];

        //finds nearest line in the current line set (if present)
        int32_t min_dist = -1;
        for (uint8_t j = 0; j < detectedLines->numLines; ++j) {
            const int32_t dist = abs((int32_t)fl->current.pos_mm - (int32_t)detectedLines->values[j].pos_mm);
            if (min_dist == -1 || dist <= min_dist) {
                min_dist = dist;
            }
        }

        if (min_dist == -1 || min_dist > MAX_LINE_JUMP_MM) {    // line is NOT present in the current line set
            fl->cntr = (fl->cntr > 0 && fl->cntr < MIN_LINE_SAMPLE_APPEAR) ? -MIN_LINE_SAMPLE_DISAPPEAR : fl->cntr == MIN_LINE_SAMPLE_APPEAR ? -1 : fl->cntr - 1;
            if (fl->cntr == -MIN_LINE_SAMPLE_DISAPPEAR) {
                remove_line(lineFilter, i);
            }
        } // else: line has been found in the current line set -> already handled
    }

    // fills filtered line array from stored ones
    filteredLines->numLines = 0;
    for (uint8_t i = 0; i < lineFilter->numLines && filteredLines->numLines < MAX_NUM_LINES; ++i) {
        const filteredLine_t *fl = &lineFilter->values[i];
        if (fl->cntr == MIN_LINE_SAMPLE_APPEAR || (fl->cntr < 0 && fl->cntr > -MIN_LINE_SAMPLE_DISAPPEAR)) {
            filteredLines->values[filteredLines->numLines++] = fl->current;
        }
    }
}
