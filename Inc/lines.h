#ifndef LINES_H_
#define LINES_H_

#include "config.h"

typedef struct {
    int8_t pos_mm;
} Line;

typedef struct {
    uint8_t numLines;
    Line values[MAX_NUM_LINES];
} Lines;

#endif /* LINES_H_ */
