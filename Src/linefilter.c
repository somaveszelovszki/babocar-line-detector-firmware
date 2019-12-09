#include "linefilter.h"
#include "config.h"

#include <stdlib.h>

static void add_line(lineFilter_t *lineFilter, const linePosition_t *line) {
    if (lineFilter->numLines < MAX_NUM_FILTERED_LINES) {
        filteredLinePosition_t *fl = &lineFilter->values[lineFilter->numLines++];
        fl->current = fl->prev = *line;
        fl->cntr = 1;
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
}

void linefilter_apply(lineFilter_t *lineFilter, linePositions_t *lines) {

    static const int32_t MAX_DIFF_MM = 5;

    for (uint8_t i = 0; i < lineFilter->numLines; ++i) {
        filteredLinePosition_t *fl = &lineFilter->values[i];
        const int32_t diff = clamp((int32_t)fl->current.pos_mm - (int32_t)fl->prev.pos_mm, -MAX_DIFF_MM, MAX_DIFF_MM);
        fl->prev.pos_mm = fl->current.pos_mm;
        fl->current.pos_mm = clamp((int32_t)fl->current.pos_mm + diff, (int32_t)MIN_OPTO_POS_MM, (int32_t)MAX_OPTO_POS_MM);
    }

    // iterates through current lines, updates counters for existing ones and adds new lines to the list
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const linePosition_t *line = &lines->values[i];

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
            filteredLinePosition_t *fl = &lineFilter->values[min_idx];
            fl->current = *line;
            fl->cntr = (fl->cntr < 0 || fl->cntr == MIN_LINE_SAMPLE_APPEAR) ? MIN_LINE_SAMPLE_APPEAR : fl->cntr + 1;
        }
    }

    // iterates through previous lines, updates counters and removes lines that have disappeared
    for (uint8_t i = 0; i < lineFilter->numLines; ++i) {
        filteredLinePosition_t *fl= &lineFilter->values[i];

        //finds nearest line in the current line set (if present)
        int32_t min_dist = -1;
        for (uint8_t j = 0; j < lines->numLines; ++j) {
            const int32_t dist = abs((int32_t)fl->current.pos_mm - (int32_t)lines->values[j].pos_mm);
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

    // refills lines from stored ones
    lines->numLines = 0;
    for (uint8_t i = 0; i < lineFilter->numLines && lines->numLines < MAX_NUM_LINES; ++i) {
        const filteredLinePosition_t *fl = &lineFilter->values[i];
        if (fl->cntr == MIN_LINE_SAMPLE_APPEAR || (fl->cntr < 0 && fl->cntr > -MIN_LINE_SAMPLE_DISAPPEAR)) {
            lines->values[lines->numLines++] = fl->current;
        }
    }
}
