#ifndef LINEFILTER_H_
#define LINEFILTER_H_

#include <micro/panel/lines.h>

typedef enum {
    VALID,
    INVALID
} fitleredLineState_t;

typedef struct {
    linePosition_t line;
    int8_t cntr;
} filteredLinePosition_t;

#define MAX_NUM_FILTERED_LINES (MAX_NUM_LINES * 2)

typedef struct {
    uint8_t numLines;
    filteredLinePosition_t values[MAX_NUM_FILTERED_LINES];
} lineFilterCalc_t;

void linefilter_initialize(lineFilterCalc_t *data);

void linefilter_apply(lineFilterCalc_t *data, linePositions_t *lines);

#endif /* LINEFILTER_H_ */
