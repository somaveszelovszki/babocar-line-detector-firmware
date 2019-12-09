#include "linepos.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct {
    uint16_t value;
    uint8_t index;
} u16_indexed_t;

#define MID_OPTO_POS ((NUM_OPTOS / 2) - 0.5f)

static inline int8_t opto_idx_to_pos_mm(float optoIdx) {
    return (int8_t)round_to_int((optoIdx - MID_OPTO_POS) * DIST_OPTO_MM);
}

static inline uint8_t pos_mm_to_opto_idx(float pos_mm) {
    return (uint8_t)clamp(round_to_int(MID_OPTO_POS + pos_mm / DIST_OPTO_MM), 0, NUM_OPTOS - 1);
}

static int cmp_u16_indexed(const void *a, const void *b) {
    return ((const u16_indexed_t*)a)->value < ((const u16_indexed_t*)b)->value ? -1 :
           ((const u16_indexed_t*)a)->value > ((const u16_indexed_t*)b)->value ? 1 : 0;
}

static void calc(linePos_t *linePos, const uint8_t *meas_without_offset) {

    linePos->lines.numLines = 0;

    uint16_t pair_intensities[NUM_OPTOS - 1];
    for (uint8_t i = 0; i < NUM_OPTOS - 1; ++i) {
        pair_intensities[i] = (uint16_t)meas_without_offset[i] + (uint16_t)meas_without_offset[i + 1];
    }

    u16_indexed_t sorted_pair_intensities[NUM_OPTOS - 1];
    for (uint8_t i = 0; i < NUM_OPTOS - 1; ++i) {
        sorted_pair_intensities[i].value = pair_intensities[i];
        sorted_pair_intensities[i].index = i;
    }

    qsort(sorted_pair_intensities, NUM_OPTOS - 1, sizeof(u16_indexed_t), cmp_u16_indexed);

    uint8_t max_idx_idx = NUM_OPTOS - 2;

    while (linePos->lines.numLines < MAX_NUM_LINES) {

        const uint8_t max_pair_idx = sorted_pair_intensities[max_idx_idx].index;
        const uint16_t max_pair_intensity = pair_intensities[max_pair_idx];

        if (max_pair_intensity < 75) {
            break;
        }

        static const uint8_t GROUP_RADIUS = 1;

        const uint8_t max_idx = meas_without_offset[max_pair_idx] >= meas_without_offset[max_pair_idx + 1] ? max_pair_idx : max_pair_idx + 1;

        const uint8_t calc_start_idx = max_idx > GROUP_RADIUS ? max_idx - GROUP_RADIUS : 0;
        const uint8_t calc_end_idx   = max_idx + GROUP_RADIUS < NUM_OPTOS ? max_idx + GROUP_RADIUS : NUM_OPTOS - 1;

        uint16_t sum = 0;
        uint16_t sumW = 0;

        for (uint8_t i = calc_start_idx; i <= calc_end_idx; ++i) {
            const uint16_t m = (uint16_t)meas_without_offset[i];
            sum += m;
            sumW += m * i;
        }

        const float optoPos = (float)sumW / sum;
        const int8_t pos_mm = opto_idx_to_pos_mm(optoPos);

        bool found = false;
        for (uint8_t i = 0; i < linePos->lines.numLines; ++i) {
            if (linePos->lines.values[i].pos_mm > (pos_mm - 10) && linePos->lines.values[i].pos_mm < (pos_mm + 10)) {
                found = true;
                break;
            }
        }

        if (!found) {
            linePos->lines.values[linePos->lines.numLines].pos_mm = pos_mm;
            ++linePos->lines.numLines;
        }

        if (max_idx_idx == 0) {
            break;
        } else {
            --max_idx_idx;
        }
    }
}

void linepos_initialize(linePos_t *linePos) {
    linePos->avgBlack = linePos->avgWhite = 0;
    linePos->comparator_black = 100;
    linePos->comparator_grey = 50;
    linePos->lines.numLines = 0;
    for (uint8_t i = 0; i < MAX_NUM_LINES; ++i) {
        linePos->lines.values[i].pos_mm = 0;
    }
}

void linepos_calc(linePos_t *linePos, const uint8_t *measurements) {

    static uint8_t white_avg = 0;
    static uint8_t white_avg_cntr = 0;

    if (0 == white_avg_cntr) {
        white_avg_cntr = 100;

        uint8_t sorted_measurements[NUM_OPTOS];
        memcpy(sorted_measurements, measurements, NUM_OPTOS);
        qsort(sorted_measurements, NUM_OPTOS, sizeof(uint8_t), compare_u8);
        white_avg = sorted_measurements[NUM_OPTOS / 3];

    } else {
        --white_avg_cntr;
    }

    uint8_t meas_without_offset[NUM_OPTOS];
    for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
        meas_without_offset[i] = measurements[i] > white_avg ? measurements[i] - white_avg : 0;
    }

    // removes edge peaks on both sides
    if (meas_without_offset[0] > 40 && meas_without_offset[1] < 10) {
        meas_without_offset[0] = meas_without_offset[1];
    }

    if (meas_without_offset[NUM_OPTOS - 1] > 40 && meas_without_offset[NUM_OPTOS - 2] < 10) {
        meas_without_offset[NUM_OPTOS - 1] = meas_without_offset[NUM_OPTOS - 2];
    }

    calc(linePos, meas_without_offset);
}


void linepos_set_leds(const linePositions_t *lines, uint8_t *leds) {
    memset(leds, 0, NUM_OPTOS / 8);
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const uint8_t optoIdx = pos_mm_to_opto_idx(lines->values[i].pos_mm);
        leds[(NUM_OPTOS / 8 - 1) - optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
    }
}
