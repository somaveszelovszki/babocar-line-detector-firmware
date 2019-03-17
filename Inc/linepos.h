#ifndef LINEPOS_H_
#define LINEPOS_H_

#include "common.h"

#include <assert.h>

#define MAX_LINES 3

typedef struct {
    float positions_cm[MAX_LINES];
    uint8_t numLines;
} Lines;

typedef struct {
    uint8_t comparator; // Comparator level for separating black and white.
    uint8_t avgWhite;   // Average white value of the previous measurement.
    uint8_t avgBlack;   // Average black value of the previous measurement.
    Lines lines;
} LinePosCalc;

void linepos_calc(LinePosCalc *data, const uint8_t *measurements);

void linepos_set_leds(LinePosCalc *data, uint8_t *leds);

#endif /* LINEPOS_H_ */
