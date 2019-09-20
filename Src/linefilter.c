#include "linefilter.h"
#include "config.h"

#include <stdlib.h>

static void add_line(lineFilterCalc_t *data, const linePosition_t *line) {
    if (data->numLines < MAX_NUM_FILTERED_LINES) {
        filteredLinePosition_t *fl = &data->values[data->numLines++];
        fl->line = *line;
        fl->cntr = 1;
    }
}

static void remove_line(lineFilterCalc_t *data, uint8_t idx) {
    for (uint8_t i = idx; i < data->numLines - 1; ++i) {
        data->values[i] = data->values[i + 1];
    }
    --data->numLines;
}

void linefilter_initialize(lineFilterCalc_t *data) {
    data->numLines = 0;
}

void linefilter_apply(lineFilterCalc_t *data, linePositions_t *lines) {

    // iterates through current lines, updates counters for existing ones and adds new lines to the list
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const linePosition_t *line = &lines->values[i];

        //finds nearest line in the previous line set (if present)
        uint8_t min_idx = 255;
        int32_t min_dist = -1;
        for (uint8_t j = 0; j < data->numLines; ++j) {
            const int32_t dist = abs(line->pos_mm - data->values[j].line.pos_mm);
            if (min_dist == -1 || dist <= min_dist) {
                min_dist = dist;
                min_idx = j;
            }
        }

        if (min_dist == -1 || min_dist > MAX_LINE_JUMP_MM) {    // line is NOT present in the previous line set
            add_line(data, line);
        } else {    // line has been found in the previous line set
            filteredLinePosition_t *fl = &data->values[min_idx];
            fl->line = *line;
            fl->cntr = (fl->cntr < 0 || fl->cntr == MIN_LINE_SAMPLE_APPEAR) ? MIN_LINE_SAMPLE_APPEAR : fl->cntr + 1;
        }
    }

    // iterates through previous lines, updates counters and removes lines that have disappeared
    for (uint8_t i = 0; i < data->numLines; ++i) {
        filteredLinePosition_t *fl= &data->values[i];

        //finds nearest line in the current line set (if present)
        int32_t min_dist = -1;
        for (uint8_t j = 0; j < lines->numLines; ++j) {
            const int32_t dist = abs(fl->line.pos_mm - lines->values[j].pos_mm);
            if (min_dist == -1 || dist <= min_dist) {
                min_dist = dist;
            }
        }

        if (min_dist == -1 || min_dist > MAX_LINE_JUMP_MM) {    // line is NOT present in the current line set
            fl->cntr = (fl->cntr > 0 && fl->cntr < MIN_LINE_SAMPLE_APPEAR) ? MIN_LINE_SAMPLE_DISAPPEAR : fl->cntr == MIN_LINE_SAMPLE_APPEAR ? -1 : fl->cntr - 1;
            if (fl->cntr == MIN_LINE_SAMPLE_DISAPPEAR) {
                remove_line(data, i);
            }
        } // else: line has been found in the current line set -> already handled
    }

    // refills lines from stored ones
    lines->numLines = 0;
    for (uint8_t i = 0; i < data->numLines && lines->numLines < MAX_NUM_LINES; ++i) {
        const filteredLinePosition_t *fl = &data->values[i];
        if (fl->cntr == MIN_LINE_SAMPLE_APPEAR) {
            lines->values[lines->numLines++] = fl->line;
        }
    }
}
