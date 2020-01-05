#ifndef LINEFILTER_H_
#define LINEFILTER_H_

#include <micro/panel/lines.h>

typedef enum {
    VALID,
    INVALID
} fitleredLineState_t;

typedef struct {
    line_t current;
    line_t prev;
    int16_t cntr;
} filteredLine_t;

#define MAX_NUM_FILTERED_LINES (MAX_NUM_LINES * 2)

typedef struct {
    uint8_t numLines;
    filteredLine_t values[MAX_NUM_FILTERED_LINES];
    uint8_t lastLineIdx;
} lineFilter_t;

void linefilter_initialize(lineFilter_t *lineFilter);

void linefilter_apply(lineFilter_t *lineFilter, lines_t *lines);

#endif /* LINEFILTER_H_ */
