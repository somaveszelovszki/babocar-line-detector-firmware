#ifndef LINEFILTER_H_
#define LINEFILTER_H_

#include <micro/panel/lines.h>

typedef enum {
    VALID,
    INVALID
} fitleredLineState_t;

typedef struct {
    linePosition_t current;
    linePosition_t prev;
    int16_t cntr;
} filteredLinePosition_t;

#define MAX_NUM_FILTERED_LINES (MAX_NUM_LINES * 2)

typedef struct {
    uint8_t numLines;
    filteredLinePosition_t values[MAX_NUM_FILTERED_LINES];
} lineFilter_t;

void linefilter_initialize(lineFilter_t *lineFilter);

void linefilter_apply(lineFilter_t *lineFilter, linePositions_t *lines);

#endif /* LINEFILTER_H_ */
