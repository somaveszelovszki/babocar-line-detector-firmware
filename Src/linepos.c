#include "linepos.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>

static const float DIST_OPTO_MM = 224.72f / 31;
static const float MID_OPTO_POS = NUM_OPTOS / 2 - 0.5f;

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
    30, // 16
    30, // 17
    30, // 18
    30, // 19
    30, // 20
    30, // 21
    30, // 22
    30, // 23
    12, // 24
    12, // 25
    12, // 26
    12, // 27
    12, // 28
    12, // 29
    12, // 30
    12  // 31
};

static void calc(linePosCalc_t *data, const uint8_t *meas_without_offset,
    const uint8_t white_avg, const uint8_t black_delta, const uint8_t grey_delta, const uint8_t minBlack, const uint8_t minDark) {

    typedef enum {
        Color_WHITE = 0,
        Color_GREY,
        Color_BLACK
    } color_t;

    uint32_t sumWhite = 0;      // The sum of the WHITE measurements (under the GREY comparator level).
    uint8_t cntrWhite = 0;      // Counts the WHITE values.

    uint32_t sumBlack = 0;      // The sum of the BLACK measurements (above the BLACK comparator level).
    uint8_t cntrBlack = 0;      // Counts the BLACK values.

    uint32_t sumDarkLocal = 0;  // The sum of the GREY or BLACK measurements corresponding to the same line.
    uint32_t sumDarkLocalW = 0; // The weighted sum of the GREY or BLACK measurements corresponding to the same line.
    uint8_t cntrBlackLocal = 0; // Counts the BLACK values corresponding to the same line.
    uint8_t cntrDarkLocal = 0;  // Counts the dark values corresponding to the same line.

    color_t prevColor = Color_WHITE;

    data->lines.numLines = 0;

    for (uint8_t i = 0; i < NUM_OPTOS + 1; ++i) {
        const uint8_t meas = i < NUM_OPTOS ? meas_without_offset[i] : 0;
        const color_t color = meas >= white_avg + black_delta ? Color_BLACK : meas >= white_avg + grey_delta ? Color_GREY : Color_WHITE;

        if (color >= Color_GREY) {
            if (prevColor == Color_WHITE) { // rising edge - new line
                sumDarkLocal = 0;
                sumDarkLocalW = 0;
                cntrBlackLocal = 0;
                cntrDarkLocal = 0;
            }

            if (color == Color_BLACK) {
                ++cntrBlackLocal;
                ++cntrBlack;
                sumBlack += meas;
            }

            sumDarkLocal += meas;
            sumDarkLocalW += meas * (uint32_t)i;
            ++cntrDarkLocal;

        } else {
            if (prevColor >= Color_GREY) {  // falling edge - line ended
                if (cntrBlackLocal >= minBlack && cntrDarkLocal >= minDark) {
                    const float optoPos = (float)sumDarkLocalW / sumDarkLocal;
                    if (data->lines.numLines < MAX_NUM_LINES) {
                        data->lines.values[data->lines.numLines++].pos_mm = opto_idx_to_pos_mm(optoPos);
                    }
                }
            }

            sumWhite += meas;
            ++cntrWhite;
        }

        prevColor = color;
    }
}

void linepos_initialize(linePosCalc_t *data) {
    data->avgBlack = data->avgWhite = 0;
    data->comparator_black = 100;
    data->comparator_grey = 50;
    data->lines.numLines = 0;
    for (uint8_t i = 0; i < MAX_NUM_LINES; ++i) {
        data->lines.values[i].pos_mm = 0;
    }
}

void linepos_calc(linePosCalc_t *data, const uint8_t *measurements) {

    uint8_t avg_, min_, max_;

    uint8_t meas_without_offset[NUM_OPTOS];
    for (uint32_t i = 0; i < NUM_OPTOS; ++i) {
        meas_without_offset[i] = measurements[i] > OFFSETS[i] ? measurements[i] - OFFSETS[i] : 0;
    }

    array_average_min_max(meas_without_offset, NUM_OPTOS, &avg_, &min_, &max_);

    uint8_t sorted_measurements[NUM_OPTOS];
    memcpy(sorted_measurements, meas_without_offset, NUM_OPTOS);
    sort(sorted_measurements, NUM_OPTOS);
    const uint8_t white_avg = sorted_measurements[(uint32_t)(NUM_OPTOS * 0.3f)];

    calc(data, meas_without_offset, white_avg, 70, 20, 0, 2);

    if (!data->lines.numLines) {
        calc(data, meas_without_offset, white_avg, 50, 10, 1, 0);
    }
}


void linepos_set_leds(const linePositions_t *lines, uint8_t *leds) {
    memset(leds, 0, NUM_OPTOS / 8);
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const uint8_t optoIdx = pos_mm_to_opto_idx(lines->values[i].pos_mm);
        leds[(NUM_OPTOS / 8 - 1) - optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
    }
}
