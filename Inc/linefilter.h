#ifndef LINEFILTER_H_
#define LINEFILTER_H_

#include <micro/panel/lines.h>

typedef enum {
    VALID,
    INVALID
} FitleredLineState;

typedef struct {
    Line line;
    int8_t cntr;
} FilteredLine;

#define MAX_NUM_FILTERED_LINES (MAX_NUM_LINES * 2)

typedef struct {
    uint8_t numLines;
    FilteredLine values[MAX_NUM_FILTERED_LINES];
} LineFilterCalc;

void linefilter_initialize(LineFilterCalc *data);

void linefilter_apply(LineFilterCalc *data, Lines *lines);

#endif /* LINEFILTER_H_ */
