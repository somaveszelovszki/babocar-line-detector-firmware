#ifndef LINEPOS_H_
#define LINEPOS_H_

#include <micro/panel/lines.h>

typedef struct {
    uint8_t comparator_black;   // Comparator level for separating BLACK.
    uint8_t comparator_grey;    // Comparator level for separating GREY.
    uint8_t avgWhite;           // Average WHITE value of the previous measurement.
    uint8_t avgBlack;           // Average BLACK value of the previous measurement.
    linePositions_t lines;      // The detected lines.
} linePos_t;

void linepos_initialize(linePos_t *linePos);

void linepos_calc(linePos_t *linePos, const uint8_t *measurements);

void linepos_set_leds(const linePositions_t *lines, uint8_t *leds);

#endif /* LINEPOS_H_ */
