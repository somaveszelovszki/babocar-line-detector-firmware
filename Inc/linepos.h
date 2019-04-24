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
    uint8_t comparator_black;   // Comparator level for separating BLACK.
    uint8_t comparator_grey;    // Comparator level for separating GREY.
    uint8_t avgWhite;           // Average WHITE value of the previous measurement.
    uint8_t avgBlack;           // Average BLACK value of the previous measurement.
    Lines lines;                // The detected lines.
} LinePosCalc;

void linepos_initialize(LinePosCalc *data);

void linepos_calc(LinePosCalc *data, const uint8_t *measurements);

void linepos_set_leds(const Lines *lines, uint8_t *leds);

void linepos_set_display(const Lines *lines, char *str);

#endif /* LINEPOS_H_ */
