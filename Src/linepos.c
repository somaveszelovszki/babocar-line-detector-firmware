#include "linepos.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define MID_OPTO_POS ((NUM_OPTOS / 2) - 0.5f)

static inline int8_t opto_idx_to_pos_mm(float optoIdx) {
    return (int8_t)round_to_int((optoIdx - MID_OPTO_POS) * DIST_OPTO_MM);
}

static inline uint8_t pos_mm_to_opto_idx(float pos_mm) {
    return (uint8_t)clamp(round_to_int(MID_OPTO_POS + pos_mm / DIST_OPTO_MM), 0, NUM_OPTOS - 1);
}

static const uint8_t OFFSETS[NUM_OPTOS] = {
    12, // 0
    12, // 1
    12, // 2
    12, // 3
    12, // 4
    12, // 5
    12, // 6
    12, // 7
    12, // 8
    12, // 9
    12, // 10
    12, // 11
    12, // 12
    12, // 13
    12, // 14
    12, // 15
    12, // 16
    12, // 17
    12, // 18
    12, // 19
    12, // 20
    12, // 21
    12, // 22
    12, // 23
    12, // 24
    12, // 25
    12, // 26
    12, // 27
    12, // 28
    12, // 29
    12, // 30
    12  // 31
};

static const float MULTIPLIERS[NUM_OPTOS] = {
    1.0f, // 0
    1.0f, // 1
    1.0f, // 2
    1.0f, // 3
    1.0f, // 4
    1.0f, // 5
    1.0f, // 6
    1.0f, // 7
    1.0f, // 8
    1.0f, // 9
    1.0f, // 10
    1.0f, // 11
    1.0f, // 12
    1.0f, // 13
    1.0f, // 14
    1.0f, // 15
    2.0f, // 16
    2.0f, // 17
    2.0f, // 18
    2.0f, // 19
    2.0f, // 20
    2.0f, // 21
    2.0f, // 22
    2.0f, // 23
    1.0f, // 24
    1.0f, // 25
    1.0f, // 26
    1.0f, // 27
    1.0f, // 28
    1.0f, // 29
    1.0f, // 30
    1.0f  // 31
};

static void calc(linePos_t *linePos, const uint8_t *meas_without_offset, const uint8_t *sorted_indexes, const uint8_t white_avg) {

    linePos->lines.numLines = 0;

    if (white_avg < 25) {

        uint8_t meas[NUM_OPTOS];
        for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
            meas[i] = meas_without_offset[i] > white_avg ? meas_without_offset[i] - white_avg : 0;
        }

        uint16_t pair_intensities[NUM_OPTOS - 1];
        for (uint8_t i = 0; i < NUM_OPTOS - 1; ++i) {
            pair_intensities[i] = (uint16_t)meas[i] + (uint16_t)meas[i + 1];
        }

        uint16_t sorted_pair_intensities[NUM_OPTOS - 1];
        memcpy(sorted_pair_intensities, pair_intensities, sizeof(uint16_t) * (NUM_OPTOS - 1));
        uint8_t sorted_pair_intensity_indexes[NUM_OPTOS - 1];
        for (uint8_t i = 0; i < NUM_OPTOS - 1; ++i) {
            sorted_pair_intensity_indexes[i] = i;
        }

        sort_u16(sorted_pair_intensities, sorted_pair_intensity_indexes, NUM_OPTOS - 1);

        uint8_t max_idx_idx = NUM_OPTOS - 2;

        uint16_t prev_max_pair_intensity = sorted_pair_intensities[NUM_OPTOS - 2];

        while (linePos->lines.numLines < MAX_NUM_LINES) {

            const uint8_t max_pair_idx = sorted_pair_intensity_indexes[max_idx_idx];
            const uint16_t max_pair_intensity = pair_intensities[max_pair_idx];

            if (max_pair_intensity < prev_max_pair_intensity / 4 || max_pair_intensity < 25) {
                break;
            }

            static const uint8_t GROUP_RADIUS = 1;

            const uint8_t max_idx = meas[max_pair_idx] >= meas[max_pair_idx + 1] ? max_pair_idx : max_pair_idx + 1;

            const uint8_t calc_start_idx = max_idx > GROUP_RADIUS ? max_idx - GROUP_RADIUS : 0;
            const uint8_t calc_end_idx   = max_idx + GROUP_RADIUS < NUM_OPTOS ? max_idx + GROUP_RADIUS : NUM_OPTOS - 1;

            uint16_t sum = 0;
            uint16_t sumW = 0;

            for (uint8_t i = calc_start_idx; i <= calc_end_idx; ++i) {
                const uint16_t m = (uint16_t)meas[i];
                sum += m;
                sumW += m * i;
            }

            const float optoPos = (float)sumW / sum;
            const int8_t pos_mm = opto_idx_to_pos_mm(optoPos);

            bool found = false;
            for (uint8_t i = 0; i < linePos->lines.numLines; ++i) {
                if (linePos->lines.values[i].pos_mm > pos_mm - 10 && linePos->lines.values[i].pos_mm < pos_mm + 10) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                linePos->lines.values[linePos->lines.numLines].pos_mm = opto_idx_to_pos_mm(optoPos);
                ++linePos->lines.numLines;
            }

            if (max_idx_idx == 0) {
                break;
            } else {
                --max_idx_idx;
            }

            prev_max_pair_intensity = max_pair_intensity;
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

    uint8_t avg_, min_, max_;

    uint8_t meas_without_offset[NUM_OPTOS];
    for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
        meas_without_offset[i] = measurements[i] > OFFSETS[i] ? measurements[i] - OFFSETS[i] : 0;
        meas_without_offset[i] = meas_without_offset[i] * MULTIPLIERS[i] >= 255.0f ? 255 : meas_without_offset[i] * MULTIPLIERS[i];
    }

    if (meas_without_offset[0] > 40 && meas_without_offset[1] < 10) {
        meas_without_offset[0] = meas_without_offset[1];
    }

    array_average_min_max(meas_without_offset, NUM_OPTOS, &avg_, &min_, &max_);

    uint8_t sorted_measurements[NUM_OPTOS];
    memcpy(sorted_measurements, meas_without_offset, NUM_OPTOS);
    uint8_t sorted_indexes[NUM_OPTOS];
    for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
        sorted_indexes[i] = i;
    }
    sort_u8(sorted_measurements, sorted_indexes, NUM_OPTOS);
    const uint8_t white_avg = sorted_measurements[(uint32_t)(NUM_OPTOS * 0.3f)];

    calc(linePos, meas_without_offset, sorted_indexes, white_avg);
}


void linepos_set_leds(const linePositions_t *lines, uint8_t *leds) {
    memset(leds, 0, NUM_OPTOS / 8);
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const uint8_t optoIdx = pos_mm_to_opto_idx(lines->values[i].pos_mm);
        leds[(NUM_OPTOS / 8 - 1) - optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
    }
}
