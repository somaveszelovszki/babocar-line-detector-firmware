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
    24,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    30,
    30,
    30,
    30,
    30,
    30,
    30,
    30,
    12,
    12,
    12,
    12,
    12,
    12,
    12,
    12
};

void linepos_initialize(LinePosCalc *data) {
    data->avgBlack = data->avgWhite = 0;
    data->comparator_black = 100;
    data->comparator_grey = 50;
    data->lines.numLines = 0;
    for (uint8_t i = 0; i < MAX_NUM_LINES; ++i) {
        data->lines.values[i].pos_mm = 0;
    }
}

void linepos_calc(LinePosCalc *data, const uint8_t *measurements) {

    typedef enum {
        Color_WHITE = 0,
        Color_GREY,
        Color_BLACK
    } Color;

    static const uint8_t black_delta = 70;
    static const uint8_t grey_delta = 30;

    uint32_t sumWhite = 0;      // The sum of the WHITE measurements (under the GREY comparator level).
    uint8_t cntrWhite = 0;      // Counts the WHITE values.

    uint32_t sumBlack = 0;      // The sum of the BLACK measurements (above the BLACK comparator level).
    uint8_t cntrBlack = 0;      // Counts the BLACK values.

    uint32_t sumDarkLocal = 0;  // The sum of the GREY or BLACK measurements corresponding to the same line.
    uint32_t sumDarkLocalW = 0; // The weighted sum of the GREY or BLACK measurements corresponding to the same line.
    uint8_t cntrBlackLocal = 0; // Counts the BLACK values corresponding to the same line.
    uint8_t cntrDarkLocal = 0;  // Counts the dark values corresponding to the same line.

    Color prevColor = Color_WHITE;

    data->lines.numLines = 0;

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

    if (max_ - min_ < black_delta) { // no peaks
        return;
    }

    for (uint8_t i = 0; i < NUM_OPTOS + 1; ++i) {
        const uint8_t meas = i < NUM_OPTOS ? meas_without_offset[i] : 0;
        const Color color = meas >= white_avg + black_delta ? Color_BLACK : meas >= white_avg + grey_delta ? Color_GREY : Color_WHITE;

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
                if (cntrBlackLocal >= 1) {
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


void linepos_set_leds(const Lines *lines, uint8_t *leds) {
    memset(leds, 0, NUM_OPTOS / 8);
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const uint8_t optoIdx = pos_mm_to_opto_idx(lines->values[i].pos_mm);
        leds[(NUM_OPTOS / 8 - 1) - optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
    }
}
