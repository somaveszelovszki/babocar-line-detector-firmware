#ifndef LINEPOS_H_
#define LINEPOS_H_

#include "common.h"

#include <assert.h>

#define MAX_LINES 3

typedef struct {
    uint8_t numLines;
    int8_t positions_mm[MAX_LINES];
} Lines;

typedef struct {
    uint8_t comparator_black;   // Comparator level for separating black.
    uint8_t comparator_grey;    // Comparator level for separating grey.
    uint8_t avgWhite;   // Average white value of the previous measurement.
    uint8_t avgDark;   // Average dark value of the previous measurement.
    Lines lines;
} LinePosCalc;

void linepos_initialize(LinePosCalc *data);

void linepos_calc(LinePosCalc *data, const uint8_t *measurements);

void linepos_set_leds(LinePosCalc *data, uint8_t *leds);

#endif /* LINEPOS_H_ */
