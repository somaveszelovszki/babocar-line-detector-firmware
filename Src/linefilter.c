#include "linefilter.h"
#include "config.h"

#include <micro/math/numeric.h>

#include <stdlib.h>

static void add_line(lineFilter_t *lineFilter, const line_t *line) {
    if (lineFilter->numLines < MAX_NUM_FILTERED_LINES) {
        filteredLine_t *fl = &lineFilter->values[lineFilter->numLines++];
        fl->current.pos_mm = fl->prev.pos_mm = line->pos_mm;
        fl->current.id = fl->prev.id = (lineFilter->lastLineIdx = 255 == lineFilter->lastLineIdx ? 1 : lineFilter->lastLineIdx + 1);
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
    lineFilter->lastLineIdx = 0;
}

void linefilter_apply(lineFilter_t *lineFilter, lines_t *lines) {

    static const int8_t MAX_OPTO_POS_MM = (int8_t)(OPTO_ARRAY_LENGTH_MM / 2);
    static const int8_t MIN_OPTO_POS_MM = (int8_t)(-OPTO_ARRAY_LENGTH_MM / 2);

    for (uint8_t i = 0; i < lineFilter->numLines; ++i) {
        filteredLine_t *fl = &lineFilter->values[i];
        const int32_t diff = CLAMP(fl->current.pos_mm - fl->prev.pos_mm, -MAX_LINE_JUMP_MM, MAX_LINE_JUMP_MM);
        fl->prev.pos_mm = fl->current.pos_mm;
        fl->current.pos_mm = CLAMP(fl->current.pos_mm + diff, MIN_OPTO_POS_MM, MAX_OPTO_POS_MM);

        for (uint8_t j = 0; j < lineFilter->numLines; ++j) {
            if (i != j) {
                filteredLine_t *fl2 = &lineFilter->values[j];
                if (abs((int32_t)fl->current.pos_mm - (int32_t)fl2->current.pos_mm) < MIN_LINE_DIST_MM) {
                    fl->current.pos_mm = CLAMP(
                        (fl->current.pos_mm > fl2->current.pos_mm ?
                            fl2->current.pos_mm + MIN_LINE_DIST_MM :
                            fl2->current.pos_mm - MIN_LINE_DIST_MM),
                        MIN_OPTO_POS_MM,
                        MAX_OPTO_POS_MM);
                }
            }
        }
    }

    // iterates through current lines, updates counters for existing ones and adds new lines to the list
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const line_t *line = &lines->values[i];

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
            fl->current = *line;
            fl->cntr = (fl->cntr < 0 || fl->cntr == MIN_LINE_SAMPLE_APPEAR) ? MIN_LINE_SAMPLE_APPEAR : fl->cntr + 1;
        }
    }

    // iterates through previous lines, updates counters and removes lines that have disappeared
    for (uint8_t i = 0; i < lineFilter->numLines; ++i) {
        filteredLine_t *fl= &lineFilter->values[i];

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
        const filteredLine_t *fl = &lineFilter->values[i];
        if (fl->cntr == MIN_LINE_SAMPLE_APPEAR || (fl->cntr < 0 && fl->cntr > -MIN_LINE_SAMPLE_DISAPPEAR)) {
            lines->values[lines->numLines++] = fl->current;
        }
    }
}
